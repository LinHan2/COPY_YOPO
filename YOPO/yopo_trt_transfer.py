"""
    将yopo模型转换为Tensorrt
    prepare:
        1 pip install -U nvidia-tensorrt --index-url https://pypi.ngc.nvidia.com
        2 git clone https://github.com/NVIDIA-AI-IOT/torch2trt
          cd torch2trt
          python setup.py install
"""

import os
import argparse
import sys
import time
import numpy as np
import torch
from torch2trt import torch2trt
from ruamel.yaml import YAML
from config.config import cfg
import time
from policy.yopo_network import YopoNetwork


def parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trial", type=int, default=1, help="trial number")
    parser.add_argument("--epoch", type=int, default=50, help="epoch number")
    parser.add_argument("--dir", type=str, default='yopo_trt.pth', help="output file name")
    return parser


if __name__ == "__main__":
    args = parser().parse_args()
    base_dir = os.path.dirname(os.path.abspath(__file__))
    weight = base_dir + "/saved/YOPO_{}/epoch{}.pth".format(args.trial, args.epoch)

    print("Loading Network...")
    device = "cuda" if torch.cuda.is_available() else "cpu"
    state_dict = torch.load(weight, weights_only=True)
    policy = YopoNetwork(horizon_num=cfg["horizon_num"], vertical_num=cfg["vertical_num"])
    policy.load_state_dict(state_dict)
    policy = policy.to(device)
    policy.eval()

    # 准备输入
    print("Preparing inputs...")
    
    # 深度图输入: [batch, channels, height, width]
    depth = np.zeros(shape=[1, 1, 96, 160], dtype=np.float32)
    depth_in = torch.from_numpy(depth).to(device)
    
    # 观测输入 - 尝试不同格式找到正确的
    obs_formats = [
        ("flat", [1, 9]),
        ("4d_primitives", [1, 9, vertical_num, horizon_num]),
        ("3d_time_series", [1, 9, total_primitives]),
    ]
    
    successful_format = None
    obs_in = None
    
    for format_name, obs_shape in obs_formats:
        try:
            print(f"Trying obs format '{format_name}' with shape: {obs_shape}")
            obs = np.zeros(shape=obs_shape, dtype=np.float32)
            obs_test = torch.from_numpy(obs).to(device)
            
            # 测试前向传播
            with torch.no_grad():
                endstate, score = policy(depth_in, obs_test)
                print(f"✓ Success with format '{format_name}'")
                print(f"  Output shapes - endstate: {endstate.shape}, score: {score.shape}")
                
                successful_format = format_name
                obs_in = obs_test
                break
                
        except Exception as e:
            print(f"✗ Failed with format '{format_name}': {e}")
            continue
    
    if obs_in is None:
        print("Could not find compatible input format")
        return
    
    print(f"Using successful format: {successful_format}")
    print(f"Final input shapes - depth: {depth_in.shape}, obs: {obs_in.shape}")

    # 分析网络输出的实际形状
    print("\nAnalyzing network output structure...")
    with torch.no_grad():
        endstate, score = policy(depth_in, obs_in)
        print(f"Network output shapes:")
        print(f"  endstate: {endstate.shape} (elements: {endstate.numel()})")
        print(f"  score: {score.shape} (elements: {score.numel()})")
        
        # 检查是否可以reshape为期望的维度
        expected_endstate_shape = (1, 9, vertical_num, horizon_num)
        expected_score_shape = (1, vertical_num, horizon_num)
        
        try:
            endstate_reshaped = endstate.view(expected_endstate_shape)
            score_reshaped = score.view(expected_score_shape)
            print(f"✓ Can reshape to expected dimensions:")
            print(f"  endstate: {endstate_reshaped.shape}")
            print(f"  score: {score_reshaped.shape}")
        except Exception as e:
            print(f"✗ Cannot reshape to expected dimensions: {e}")

    print("\nTensorRT Transfer...")
    try:
        # 转换为TensorRT
        model_trt = torch2trt(policy, [depth_in, obs_in], fp16_mode=True)
        torch.save(model_trt.state_dict(), args.dir)
        print(f"TensorRT model saved to: {args.dir}")
    except Exception as e:
        print(f"TensorRT conversion failed: {e}")
        print("You can still use the PyTorch model with --use_tensorrt=0")
        return

    print("Evaluation...")
    try:
        # Warm up
        with torch.no_grad():
            endstate_trt, score_trt = model_trt(depth_in, obs_in)
            endstate, score = policy(depth_in, obs_in)
        torch.cuda.synchronize()

        # 性能测试
        num_runs = 100
        
        # PyTorch延迟
        torch.cuda.synchronize()
        torch_start = time.time()
        for _ in range(num_runs):
            with torch.no_grad():
                endstate, score = policy(depth_in, obs_in)
        torch.cuda.synchronize()
        torch_end = time.time()
        torch_latency = (torch_end - torch_start) / num_runs

        # TensorRT延迟
        torch.cuda.synchronize()
        trt_start = time.time()
        for _ in range(num_runs):
            with torch.no_grad():
                endstate_trt, score_trt = model_trt(depth_in, obs_in)
        torch.cuda.synchronize()
        trt_end = time.time()
        trt_latency = (trt_end - trt_start) / num_runs

        # 计算误差
        endstate_error = torch.mean(torch.abs(endstate - endstate_trt))
        score_error = torch.mean(torch.abs(score - score_trt))

        print(f"Performance Results:")
        print(f"  PyTorch Latency: {1000 * torch_latency:.3f} ms")
        print(f"  TensorRT Latency: {1000 * trt_latency:.3f} ms")
        print(f"  Speedup: {torch_latency / trt_latency:.2f}x")
        print(f"  Transfer Endstate Error: {endstate_error.item():.6f}")
        print(f"  Transfer Score Error: {score_error.item():.6f}")
        
        print("TensorRT conversion completed successfully!")
        
    except Exception as e:
        print(f"Error during evaluation: {e}")


if __name__ == "__main__":
    main()

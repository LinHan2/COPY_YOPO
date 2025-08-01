"""
    将yopo模型转换为Tensorrt
    prepare:
        1 pip install -U nvidia-tensorrt --index-url https://pypi.ngc.nvidia.com
        2 git clone https://github.com/NVIDIA-AI-IOT/torch2trt
          cd torch2trt
          python setup.py install
"""

import argparse
import os
import sys
import numpy as np
import torch
from torch2trt import torch2trt
from ruamel.yaml import YAML
import time

# 添加当前目录到Python路径，解决导入问题
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, current_dir)

from policy.yopo_network import YopoNetwork


def parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trial", type=int, default=1, help="trial number")
    parser.add_argument("--epoch", type=int, default=50, help="epoch number")
    parser.add_argument("--dir", type=str, default='yopo_trt.pth', help="output file name")
    return parser


def main():
    args = parser().parse_args()
    base_dir = os.path.dirname(os.path.abspath(__file__))
    cfg = YAML().load(open(os.path.join(base_dir, "config/traj_opt.yaml"), 'r'))
    weight = base_dir + "/saved/YOPO_{}/epoch{}.pth".format(args.trial, args.epoch)

    print("Loading Network...")
    device = "cuda" if torch.cuda.is_available() else "cpu"
    
    # 检查模型文件
    if not os.path.exists(weight):
        print(f"Error: Model file not found: {weight}")
        return
    
    # 从配置文件获取维度信息
    horizon_num = cfg['horizon_num']  # 5
    vertical_num = cfg['vertical_num']  # 3
    total_primitives = horizon_num * vertical_num  # 15
    
    print(f"Configuration:")
    print(f"  horizon_num: {horizon_num}")
    print(f"  vertical_num: {vertical_num}")
    print(f"  total_primitives: {total_primitives}")
    
    # 创建网络 - 使用与预训练模型匹配的参数
    observation_dim = 9
    hidden_state = 64
    output_dim = 10  # 修复：使用原始的输出维度，不是 10 * total_primitives
    
    print(f"Creating network with:")
    print(f"  observation_dim: {observation_dim}")
    print(f"  output_dim: {output_dim}")
    print(f"  hidden_state: {hidden_state}")
    
    try:
        policy = YopoNetwork(
            observation_dim=observation_dim,
            output_dim=output_dim,
            hidden_state=hidden_state
        )
        print("Network created successfully")
    except Exception as e:
        print(f"Error creating network: {e}")
        return

    # 加载模型权重
    try:
        state_dict = torch.load(weight, weights_only=True, map_location=device)
        print("Model weights loaded successfully")
    except TypeError:
        # 处理旧版本PyTorch
        state_dict = torch.load(weight, map_location=device)
        print("Model weights loaded (fallback mode)")
    except Exception as e:
        print(f"Error loading weights: {e}")
        return

    # 加载权重
    try:
        policy.load_state_dict(state_dict)
        print("State dict loaded successfully")
    except Exception as e:
        print(f"Error loading state dict: {e}")
        return
    
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
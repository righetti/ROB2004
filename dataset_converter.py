#!/usr/bin/env python3
"""
Convert one LeRobot HF dataset to another LeRobot HF dataset with:
- State-Relative Actions (Action - State)
- Adjusted FPS
- Resized Images
"""

import argparse
import torch
import numpy as np
from pathlib import Path
from tqdm import tqdm
from torch.nn.functional import interpolate

# Import LeRobotDataset
try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    LEROBOT_AVAILABLE = True
except ImportError:
    LEROBOT_AVAILABLE = False
    print("Error: 'lerobot' library is required.")
    exit(1)

def convert_dataset(args):
    # 1. Load Source Dataset
    print(f"Loading source: {args.repo_id}")
    if args.local_dir:
        source_ds = LeRobotDataset(repo_id=args.repo_id, root=args.local_dir, video_backend='pyav')
    else:
        source_ds = LeRobotDataset(repo_id=args.repo_id, video_backend='pyav')

    # 2. Determine Output Settings
    source_fps = source_ds.fps
    target_fps = args.target_fps if args.target_fps else source_fps
    ds_ratio = max(1, int(source_fps // target_fps))
    
    print(f"FPS: {source_fps} -> {target_fps} (Subsample: {ds_ratio})")
    
    # Define features for the new dataset based on the source
    # We copy features but might need to adjust shapes if resizing
    features = source_ds.features
    
    # Update Image Features if resizing
    if args.width and args.height:
        for key, feat in features.items():
            if key == args.image_key and 'shape' in feat:
                # Shape is typically (height, width, channels) or (channels, height, width) 
                # LeRobot features usually expect (H, W, C) for video storage metadata
                # but let's check current convention. Usually it is (H, W, C).
                original_shape = feat['shape']
                # Preserving channel count (usually last dim in feature desc, but first in tensor)
                # We assume standard (H, W, C) for feature description
                channels = original_shape[2] 
                features[key]['shape'] = (args.height, args.width, channels)
    # 3. Create Target Dataset
    output_path = Path(args.output_dir)
    target_ds = LeRobotDataset.create(
        repo_id=args.output_repo_id,
        fps=target_fps,
        root=output_path,
        features=features,
        use_videos=True, # Efficiently store images as video
        image_writer_threads=8 # Parallel encoding
    )
    
    # State tracking
    prev_episode_idx = source_ds[0]['episode_index'].item()
    frame_idx_in_episode = 0
    
    print("Starting conversion...")
    
    for idx in tqdm(range(len(source_ds))):
        sample = source_ds[idx]
        curr_episode_idx = sample['episode_index'].item()
        
        # --- Episode Boundary Check ---
        if curr_episode_idx != prev_episode_idx:
            # Save the FINISHED episode to disk/parquet
            target_ds.save_episode()
            
            # Reset counters
            prev_episode_idx = curr_episode_idx
            frame_idx_in_episode = 0

        # --- Subsampling & Processing ---
        if frame_idx_in_episode % ds_ratio == 0:
            
            # 1. Process Image
            img_tensor = sample[args.image_key] # (C, H, W)
            
            # Pad to Square if requested
            if args.rectangular:
                C, H, W = img_tensor.shape
                max_side = max(H, W)
                img_padded = torch.zeros((C, max_side, max_side), dtype=img_tensor.dtype)
                r, c = (max_side - H)//2, (max_side - W)//2
                img_padded[:, r:r+H, c:c+W] = img_tensor
                img_to_resize = img_padded
            else:
                img_to_resize = img_tensor

            # Resize
            if args.width and args.height:
                # Interpolate expects (N, C, H, W)
                resized_img = interpolate(
                    img_to_resize.unsqueeze(0), 
                    size=(args.height, args.width), 
                    mode='bilinear', 
                    align_corners=False
                ).squeeze(0)
            else:
                resized_img = img_to_resize

            # 2. Process Action (State-Relative)
            # Formula: Rel_Action = Action - State
            action = sample[args.action_key]
            
            if args.relative_actions:
                current_state = sample[args.state_key]
                if current_state.shape != action.shape:
                     raise ValueError(f"Shape mismatch: State {current_state.shape} vs Action {action.shape}")
                
                # Logic: Vector from Current State -> Target Action
                action_rel = action - current_state
            else:
                action_rel = action

            # 3. Construct New Frame
            # We copy all other keys (like 'task_index', 'timestamp', etc.) directly
            
            # Create new sample with relative action
            new_frame = {k: v for k, v in sample.items() if k not in [args.image_key, args.action_key, 'timestamp', 'index', 'frame_index', 'task_index', 'episode_index']}
            
            # Insert modified data
            # Note: LeRobotDataset.add_frame expects Tensors or Numpy arrays
            # Images should be (C, H, W) float 0-1 or (C, H, W) uint8 0-255?
            # LeRobot add_frame handles normalization, but usually expects 0-1 float or 0-255 uint8.
            # Since we resized using torch interpolate (float), let's ensure it matches original dtype style or keeps float.
            new_frame[args.image_key] = (resized_img.permute(1,2,0).numpy()*255.0).astype(np.uint8)
            new_frame[args.action_key] = action_rel            
            # Add to buffer
            target_ds.add_frame(new_frame)

        frame_idx_in_episode += 1

    # --- Flush Final Episode ---
    target_ds.save_episode()
        
    print(f"\nConversion Complete!")
    print(f"Saved to: {args.output_dir}")
    print(f"Repo ID: {args.output_repo_id}")

def main():
    parser = argparse.ArgumentParser()
    # IO Args
    parser.add_argument('--repo_id', type=str, required=True, help="Source HF dataset")
    parser.add_argument('--output_repo_id', type=str, required=True, help="Name for new dataset")
    parser.add_argument('--output_dir', type=str, required=True, help="Local path for new dataset")
    parser.add_argument('--local_dir', type=str, default=None, help="Local path of source dataset")
    
    # Processing Args
    parser.add_argument('--target_fps', type=int, default=20)
    parser.add_argument('--width', type=int, default=512)
    parser.add_argument('--height', type=int, default=512)
    parser.add_argument('--rectangular', type=bool, default=True, help="Pad images to square before resizing")
    
    # Logic Args
    parser.add_argument('--relative_actions', action='store_true', default=True, help="Compute Action - State")
    parser.add_argument('--image_key', type=str, default='observation.images.camera_0')
    parser.add_argument('--action_key', type=str, default='action')
    parser.add_argument('--state_key', type=str, default='observation.state')

    args = parser.parse_args()
    convert_dataset(args)

if __name__ == "__main__":
    main()

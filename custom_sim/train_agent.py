"""
Training script for LiDAR-based robot navigation using Reinforcement Learning.

This script trains a PPO agent to navigate using LiDAR data in the custom simulation environment.
"""

import os
import numpy as np
import torch
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, StopTrainingOnRewardThreshold
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
import matplotlib.pyplot as plt

from rl_env import LidarNavigationEnv


def create_env(config_file='robot_world.yaml', **kwargs):
    """Create a single environment instance."""
    return LidarNavigationEnv(config_file=config_file, **kwargs)


def train_agent(
    config_file: str = 'robot_world.yaml',
    total_timesteps: int = 500000,
    n_envs: int = 4,
    learning_rate: float = 3e-4,
    n_steps: int = 2048,
    batch_size: int = 64,
    n_epochs: int = 10,
    gamma: float = 0.99,
    gae_lambda: float = 0.95,
    clip_range: float = 0.2,
    ent_coef: float = 0.01,
    vf_coef: float = 0.5,
    max_grad_norm: float = 0.5,
    target_reward: float = 80.0,
    eval_freq: int = 10000,
    save_freq: int = 50000,
    model_save_path: str = './models/',
    log_dir: str = './logs/'
):
    """
    Train a PPO agent for LiDAR navigation.
    
    Args:
        config_file: YAML configuration file for the environment
        total_timesteps: Total training timesteps
        n_envs: Number of parallel environments
        learning_rate: Learning rate for the optimizer
        n_steps: Number of steps to run for each environment per update
        batch_size: Minibatch size
        n_epochs: Number of epoch when optimizing the surrogate loss
        gamma: Discount factor
        gae_lambda: Factor for trade-off of bias vs variance for Generalized Advantage Estimator
        clip_range: Clipping parameter for PPO
        ent_coef: Entropy coefficient for the loss calculation
        vf_coef: Value function coefficient for the loss calculation
        max_grad_norm: The maximum value for the gradient clipping
        target_reward: Target average reward for early stopping
        eval_freq: Evaluate the agent every eval_freq steps
        save_freq: Save the model every save_freq steps
        model_save_path: Directory to save trained models
        log_dir: Directory for tensorboard logs
    """
    
    # Create directories
    os.makedirs(model_save_path, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)
    
    print(f"Training PPO agent with {n_envs} parallel environments...")
    print(f"Total timesteps: {total_timesteps}")
    print(f"Config file: {config_file}")
    
    # Create vectorized training environment
    env_kwargs = {
        'config_file': config_file,
        'max_steps': 1000,
        'render_mode': None  # No rendering during training
    }
    
    if n_envs > 1:
        # Use subprocess vectorization for true parallelism
        train_env = make_vec_env(
            create_env, 
            n_envs=n_envs, 
            env_kwargs=env_kwargs,
            vec_env_cls=SubprocVecEnv
        )
    else:
        train_env = DummyVecEnv([lambda: Monitor(create_env(**env_kwargs))])
    
    # Create evaluation environment
    eval_env = Monitor(create_env(config_file=config_file, render_mode=None))
    
    # Create PPO agent
    policy_kwargs = dict(
        net_arch=[512, 512],  # Two hidden layers with 512 units each
        activation_fn=torch.nn.ReLU
    )
    
    model = PPO(
        "MultiInputPolicy",
        train_env,
        learning_rate=learning_rate,
        n_steps=n_steps,
        batch_size=batch_size,
        n_epochs=n_epochs,
        gamma=gamma,
        gae_lambda=gae_lambda,
        clip_range=clip_range,
        clip_range_vf=None,
        ent_coef=ent_coef,
        vf_coef=vf_coef,
        max_grad_norm=max_grad_norm,
        use_sde=False,
        sde_sample_freq=-1,
        target_kl=None,
        tensorboard_log=log_dir,
        policy_kwargs=policy_kwargs,
        verbose=1
    )
    
    # Setup callbacks
    stop_callback = StopTrainingOnRewardThreshold(
        reward_threshold=target_reward, 
        verbose=1
    )
    
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=os.path.join(model_save_path, 'best_model'),
        log_path=log_dir,
        eval_freq=eval_freq,
        deterministic=True,
        render=False,
        callback_on_new_best=stop_callback,
        verbose=1
    )
    
    # Train the agent
    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=eval_callback,
            tb_log_name="ppo_lidar_navigation"
        )
        
        # Save final model
        final_model_path = os.path.join(model_save_path, 'final_model')
        model.save(final_model_path)
        print(f"Training completed! Final model saved to: {final_model_path}")
        
    except KeyboardInterrupt:
        print("Training interrupted by user.")
        interrupted_model_path = os.path.join(model_save_path, 'interrupted_model')
        model.save(interrupted_model_path)
        print(f"Model saved to: {interrupted_model_path}")
    
    finally:
        train_env.close()
        eval_env.close()
    
    return model


def test_agent(model_path: str, config_file: str = 'robot_world.yaml', 
               n_episodes: int = 10, render: bool = True):
    """
    Test a trained agent.
    
    Args:
        model_path: Path to the trained model
        config_file: Environment configuration file
        n_episodes: Number of episodes to test
        render: Whether to render the environment
    """
    
    print(f"Testing trained agent from: {model_path}")
    
    # Load the trained model
    model = PPO.load(model_path)
    
    # Create test environment
    test_env = create_env(config_file=config_file, 
                         render_mode='human' if render else None)
    
    episode_rewards = []
    episode_lengths = []
    success_count = 0
    
    for episode in range(n_episodes):
        obs, info = test_env.reset()
        episode_reward = 0
        episode_length = 0
        done = False
        
        print(f"\\nEpisode {episode + 1}/{n_episodes}")
        
        while not done:
            # Get action from trained model
            action, _states = model.predict(obs, deterministic=True)
            
            # Take step
            obs, reward, terminated, truncated, info = test_env.step(action)
            done = terminated or truncated
            
            episode_reward += reward
            episode_length += 1
            
            if render:
                test_env.render()
            
            # Print progress
            if episode_length % 100 == 0:
                print(f"  Step {episode_length}, Reward: {episode_reward:.2f}, "
                      f"Distance to goal: {info['distance_to_goal']:.2f}")
        
        episode_rewards.append(episode_reward)
        episode_lengths.append(episode_length)
        
        if info['goal_reached']:
            success_count += 1
            print(f"  ✓ Goal reached! Episode reward: {episode_reward:.2f}, "
                  f"Length: {episode_length} steps")
        else:
            print(f"  ✗ Episode ended. Reward: {episode_reward:.2f}, "
                  f"Length: {episode_length} steps")
    
    # Print summary statistics
    print(f"\\n=== Test Results ===")
    print(f"Success rate: {success_count}/{n_episodes} ({100*success_count/n_episodes:.1f}%)")
    print(f"Average reward: {np.mean(episode_rewards):.2f} ± {np.std(episode_rewards):.2f}")
    print(f"Average episode length: {np.mean(episode_lengths):.1f} ± {np.std(episode_lengths):.1f}")
    
    test_env.close()
    
    return {
        'success_rate': success_count / n_episodes,
        'episode_rewards': episode_rewards,
        'episode_lengths': episode_lengths
    }


def plot_training_results(log_dir: str):
    """Plot training results from tensorboard logs."""
    try:
        from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
        
        # Load tensorboard logs
        event_acc = EventAccumulator(log_dir)
        event_acc.Reload()
        
        # Extract data
        rewards = event_acc.Scalars('eval/mean_reward')
        episode_lengths = event_acc.Scalars('eval/mean_ep_length')
        
        # Create plots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        
        # Plot rewards
        steps = [x.step for x in rewards]
        reward_values = [x.value for x in rewards]
        ax1.plot(steps, reward_values)
        ax1.set_xlabel('Training Steps')
        ax1.set_ylabel('Mean Episode Reward')
        ax1.set_title('Training Progress - Reward')
        ax1.grid(True)
        
        # Plot episode lengths
        steps = [x.step for x in episode_lengths]
        length_values = [x.value for x in episode_lengths]
        ax2.plot(steps, length_values)
        ax2.set_xlabel('Training Steps')
        ax2.set_ylabel('Mean Episode Length')
        ax2.set_title('Training Progress - Episode Length')
        ax2.grid(True)
        
        plt.tight_layout()
        plt.savefig(os.path.join(log_dir, 'training_progress.png'))
        plt.show()
        
    except ImportError:
        print("Tensorboard not available. Install with: pip install tensorboard")
    except Exception as e:
        print(f"Error plotting training results: {e}")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Train or test LiDAR navigation agent')
    parser.add_argument('--mode', choices=['train', 'test', 'plot'], default='train',
                       help='Mode: train new agent, test existing agent, or plot results')
    parser.add_argument('--config', default='robot_world.yaml',
                       help='Environment configuration file')
    parser.add_argument('--model-path', default='./models/best_model',
                       help='Path to save/load model')
    parser.add_argument('--timesteps', type=int, default=500000,
                       help='Training timesteps')
    parser.add_argument('--n-envs', type=int, default=4,
                       help='Number of parallel environments')
    parser.add_argument('--episodes', type=int, default=10,
                       help='Number of test episodes')
    parser.add_argument('--no-render', action='store_true',
                       help='Disable rendering during testing')
    
    args = parser.parse_args()
    
    if args.mode == 'train':
        print("Starting training...")
        model = train_agent(
            config_file=args.config,
            total_timesteps=args.timesteps,
            n_envs=args.n_envs,
            model_save_path=os.path.dirname(args.model_path),
        )
        
    elif args.mode == 'test':
        print("Starting testing...")
        if not os.path.exists(args.model_path + '.zip'):
            print(f"Model not found at {args.model_path}. Train a model first.")
            exit(1)
        
        results = test_agent(
            model_path=args.model_path,
            config_file=args.config,
            n_episodes=args.episodes,
            render=not args.no_render
        )
        
    elif args.mode == 'plot':
        print("Plotting training results...")
        plot_training_results('./logs/ppo_lidar_navigation_1')
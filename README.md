<<<<<<< Updated upstream
# robot_description
=======
# dobot_l20_RL
Refer to the simtoolreal repository https://github.com/tylerlum/simtoolreal

# train
python isaacsim_l20/train.py --task Isaacsimenvs-SimToolReal-Direct-v0 --agent rl_games_sapg_cfg_entry_point --headless --capture_viewer --wandb_activate --wandb_project linker-hand --wandb_entity wang_jie2333-ustb env.scene.num_envs=24576 agent.params.config.expl_coef_block_size=4096

# pose_viewer
python3 -m http.server 8765
http://127.0.0.1:8765/pose_viewer_step_000462599_0077.html
>>>>>>> Stashed changes

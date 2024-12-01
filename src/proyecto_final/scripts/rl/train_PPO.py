import sys
sys.path.append("/home/laboratorio/ros_workspace/src/proyecto_final/scripts/rl")
from proyecto_final.scripts.rl.env_rob_train import ROSEnv
import os
from stable_baselines3 import PPO
from datetime import date

def train(n_cubos_max:int = 2):
    path = f"./src/proyecto_final/scripts/rl/logs/{n_cubos_max}_cubos"
    try:
        os.mkdir(path)
    except:
        pass
    env = ROSEnv(n_cubos_max, visualization=True)
    env.reset()


    model = PPO('MlpPolicy',
                env,
                tensorboard_log=path)

    # model = PPO(
            #     "MlpPolicy",
            #     env,
            #     batch_size = 50,
            #     n_steps = 1000,
            #     gae_lambda = 0.95,
            #     gamma= 0.999,
            #     n_epochs= 10,
            #     ent_coef= 0.0,
            #     learning_rate= 0.0003,
            #     clip_range= 0.18,
            #     verbose = 1,
            #     tensorboard_log = path,
            #     device='cpu'
            # )

    model.learn(total_timesteps=20, log_interval=5)
    today = date.today()
    model.save(f"src/proyecto_final/scripts/rl/agentes_entrenados/ppo_rosenv_prueba_{today}")

def test(n_cubos_max:int = 2):
    env = ROSEnv(n_cubos_max, visualization=True)

    model = PPO.load("src/proyecto_final/scripts/rl/agentes_entrenados/ppo_rosenv_prueba_0.zip")

    while True:
        obs, _ = env.reset()
        action, states = model.predict(obs)
        obs, rewards, dones, _, info = env.step(action)

if __name__ == '__main__':
    n_cubos_max = 2
    train(n_cubos_max=n_cubos_max)
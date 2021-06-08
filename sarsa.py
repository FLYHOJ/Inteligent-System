from gym import spaces
import numpy as np
import gym

env = gym.make("CliffWalking-v0")
print(env.observation_space)
print(env.action_space)
obs = env.reset()
print(obs)

class SARSA:

    def __init__(self, env):

        assert isinstance(env, gym.Env), "Environment is supposed to be gym.Env"

        self.env = env
        self.num_state = env.observation_space.n if isinstance(env.observation_sapce, gym.spaces.Discrete) else env.observation_space.shape[0]
        self.num_action = env.action_space.n if isinstance(env.action_space, gym.spaces.Discrete) else env.action_space.shape[0]

        self.q_value = np.zeros((self.num_state, self.num_action))

        self.epsilon = 0.1
        self.alpha = 0.01

    def choose_action(self, state):
        """
        """
        assert self.env.observation_space.contains(state)
        if np.random.rand() < self.epsilon:
            action = np.random.choice(self.num_action)
        else:
            action = np.argmax(self.q_value[state, :])
            print(action)
        return action

    def update(self, state, action, target):
        """
        """
        self.q_value[state, action] += self.alpha * target

    
class NStepSARSA(SARSA):

    def __init__(self, env, n=1):
        super().__init__(env)

        self.n = n
        self.strore_reward = np.zeros(self.n)

    def step(self):
        """
        """
        done = False
        obs = self.env.reset()

        while not done:
            action = self.choose_action(obs)
            


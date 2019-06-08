from multiprocessing import Process, Queue
import threading
from baselines.common.schedules import LinearSchedule
from baselines.deepq.replay_buffer import ReplayBuffer, PrioritizedReplayBuffer
import numpy as np
import time


class MemBufferThread(threading.Thread):
    # 注意可变参数概念
    def __init__(self,
                 mem_queue,
                 max_timesteps=1000000,
                 buffer_size=50000,
                 batch_size=32,
                 prioritized_replay=False,
                 prioritized_replay_alpha=0.6,
                 prioritized_replay_beta0=0.4,
                 prioritized_replay_beta_iters=None,
                 prioritized_replay_eps=1e-6):

        threading.Thread.__init__(self)
        self.mem_queue = mem_queue
        self.prioritized_replay = prioritized_replay
        self.batch_size = batch_size
        self.batch_idxes = None
        self.prioritized_replay_eps = prioritized_replay_eps

        # Create the replay buffer
        if prioritized_replay:
            self.replay_buffer = PrioritizedReplayBuffer(buffer_size, alpha=prioritized_replay_alpha)
            if prioritized_replay_beta_iters is None:
                prioritized_replay_beta_iters = max_timesteps
            self.beta_schedule = LinearSchedule(prioritized_replay_beta_iters,
                                                initial_p=prioritized_replay_beta0,
                                                final_p=1.0)
        else:
            self.replay_buffer = ReplayBuffer(buffer_size)
            self.beta_schedule = None

    def __len__(self):
        return self.replay_buffer.__len__()

    def sample(self, t):
        if self.prioritized_replay:
            experience = self.replay_buffer.sample(self.batch_size, beta=self.beta_schedule.value(t))  # 这个t的取值有待商议,
            (obses_t, actions, rewards, obses_tp1, dones, weights, self.batch_idxes) = experience
        else:
            obses_t, actions, rewards, obses_tp1, dones = self.replay_buffer.sample(self.batch_size)
            #  np.ones_like() : Return an array of ones with the same shape and type as a given array.
            weights, self.batch_idxes = np.ones_like(rewards), None

        return obses_t, actions, rewards, obses_tp1, dones, weights

    def update_priorities(self, td_errors):
        new_priorities = np.abs(td_errors) + self.prioritized_replay_eps
        self.replay_buffer.update_priorities(self.batch_idxes, new_priorities)

    def run(self):
        # flag = 1
        while True:
            if self.mem_queue.full() is True:
                print("the mem_queue is full")
            # if self.replay_buffer.__len__() >= 100000 and self.replay_buffer.__len__() % 100 == 0:  # bool(flag):
            #     # print("replay_buffer is 100000 !")
            #     print('')
            #    flag = 0
            if self.mem_queue.empty() is not True:
                single_mem = self.mem_queue.get()
                self.replay_buffer.add(single_mem[0], single_mem[1], single_mem[2], single_mem[3], single_mem[4])
            # time.sleep(0.02)  # actor大约每秒10*acnum , 性能高的不需要sleep





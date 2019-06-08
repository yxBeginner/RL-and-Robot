# import gym
#
#
# action_space = gym.spaces.Discrete(4)
# action1 = 3
# # assert action_space.contains(action1), "%r (%s) invalid"%(action1, type(action1))
# state = [0.0 for i in range(0, 14)]
# print(state)
# state2 = [0.0 for i in range(0, 15)]
# print(state2)
# state2 = state[0:14]
# print(state2)
# state2.append(100)
# print(state2)

import numpy as np
import time
import gym
# import p2os_test.src.test0624

# kk = np.array([])
# print(kk)
# a = (1, 2, 3, 4)
# b = [5, 6]
# test_list = [x for x in range(0, 100)]
# test_list2 = [x for x in range(0, 10)]
# for i in range(1,9):
#     test_list2[i] = test_list[i*10]
#
# ff = np.random.ranf()
# print(ff - 0.5)
#
#
# print(test_list)
# print(test_list2)
# print(a[:])
# # b = a[0:3]
# c = np.array(b)
# c = np.append(c, [7, 8])
# d = np.append([1, 2, 3], [[4, 5, 6], [7, 8, 9]])
# print(b)
# print(c)
# print(d)
# start_time_test = time.time()
# count = 0
#
# c = c.reshape((1, 4))
# print(c.shape)
# print(d.shape)
#
# observation_space = gym.spaces.Box(0.1, 10, (1, 16), dtype=np.dtype(np.float64))
# print(observation_space.shape)
# # observation_space_2 = gym.spaces.Box(-1, 1)
# # print(observation_space_2.shape)
# a_s = gym.spaces.Discrete(3)
# print(a_s.shape)
# while (time.time() - start_time_test) < 10:
#     ff = np.random.ranf()
#     ff2 = np.random.ranf()
#     print("ff:")
#     print(20*(ff-0.5))
#     print("ff2:")
#     print(20*(ff2-0.5))

# print(e)
#
# list_a = np.array([1, 2, 3])
# list_b = list_a
# list_b = np.append(list_b, 10)
# print(list_b)

# total_steps = 0
# trace_need = 1
#
# while total_steps < 10000:
#     total_steps += 3
#     if total_steps >= (trace_need * 1000):
#         trace_need += 1
#         print(total_steps)

a = 12.3456
print(round(a))

# yx 2018.07.03
# import sys
import tensorflow as tf
import baselines.common.tf_util as U
from baselines.deepq.build_graph import build_act, build_act_with_param_noise
# from multiprocessing import Process, Manager, Lock

# (注意只是实现功能定义,并不需要真实参数,关于Manager\Lock还有待考虑)
def actor_build(make_obs_ph, q_func, num_actions, net_list,
                scope="actor_deepq", reuse=None, param_noise=False, param_noise_filter_func=None):
    # similar with build_train(),用于创建actor的深度Q网络以及其它功能(分别运行在不同进程下,不用担心scope的问题)
    # in_list 为shared list, 用于存储trainer的q网络(np.array, 不是tensor)

    if param_noise:
        act_f = build_act_with_param_noise(make_obs_ph, q_func, num_actions, scope=scope, reuse=reuse,
                                           param_noise_filter_func=param_noise_filter_func)
    else:
        act_f = build_act(make_obs_ph, q_func, num_actions, scope=scope, reuse=reuse)
    # 与build不同, actor只需要前向传播即可
    with tf.variable_scope(scope, reuse=reuse):
        # set up placeholders
        obs_t_input = make_obs_ph("obs_t")
        # 创建Q network, 返回所有action 的q值(q_func()) , q_t是一个列表
        # q network evaluation
        q_t = q_func(obs_t_input.get(), num_actions, scope="q_func", reuse=True)  # reuse parameters from act
        # q_func_vars 得到的是q network 中所有的变量参数？tf.get_collection() 以列表的形式获取（scope下）集合中的值
        # 所以在进程间通信时,只需要传输q_func_vars就可以了
        q_func_vars = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES,
                                        scope=tf.get_variable_scope().name + "/q_func")
        # 将q_func_vars 写到update_qfunc()下就无效了,why?
        placeholder_list = []
        for item in net_list:
            placeholder_list.append(tf.placeholder(tf.float32, shape=item.shape))
        # 不要将op定义到循环下去,会造成内存泄漏
        update_qfunc_op = list()
        for var, var_target in zip(placeholder_list, q_func_vars):
            # op = tf.assign(var_target, var)
            update_qfunc_op.append(tf.assign(var_target, var))  # 操作子
        # update_qfunc_op = tf.group(*update_qfunc_op)  # group
        len_qfunc = len(update_qfunc_op)

        def update_qfunc(sess, net_list_lock):
            # 需要tf.variable_scope(scope, reuse=reuse):　
            # 或使用tf.get_default_session()(不可用上下文管理器)
            with sess.as_default():
                net_list_lock.acquire()
                # for op in update_qfunc_op:
                #     sess.run(op)
                for i in range(0, len_qfunc):
                    sess.run(update_qfunc_op[i], feed_dict={placeholder_list[i]: net_list[i]})
                    # sess.run(update_qfunc_op[i],
                    #          feed_dict={placeholder_list[i]: net_list[i+net_list_index.value*len_qfunc]})
                # for next_var_target in q_func_vars:
                #     print(next_var_target.eval(session=sess))
                # print(net_list)
                net_list_lock.release()  # 释放

        # 下面是几个不同的实现版本,或多或少的有一些问题
        # start right
        # update_qfunc_op = list()
        # for var, var_target in zip(net_list, q_func_vars):
        #     # op = tf.assign(var_target, var)
        #     update_qfunc_op.append(tf.assign(var_target, var))
        # update_qfunc_op = tf.group(*update_qfunc_op)  # group
        #
        # def update_qfunc(sess, net_list_lock):
        #     # print('update_qfunc')
        #     # 需要tf.variable_scope(scope, reuse=reuse):　
        #     # 或使用tf.get_default_session()(不可用上下文管理器)
        #     with sess.as_default():
        #         net_list_lock.acquire()
        #         sess.run(update_qfunc_op)
        #         # for i in range(0,len(update_qfunc_op)):
        #         #     sess.run(update_qfunc_op[i])
        #         # for next_var_target in q_func_vars:
        #         #     print(next_var_target.eval(session=sess))
        #         # print(net_list)
        #         net_list_lock.release()  
        #     # print('end_update_qfunc')
        # end right

        # update_qfunc_op = list()
        # for var, var_target in zip(net_list, q_func_vars): 
        #     # op = tf.assign(var_target, var)
        #     update_qfunc_op.append(tf.assign(var_target, var))

        # def update_qfunc(sess, net_list_lock):
        #     # print('update_qfunc')
        #     # 需要tf.variable_scope(scope, reuse=reuse):　
        #     # 或使用tf.get_default_session()(不可用上下文管理器)
        #     with sess.as_default():
        #         net_list_lock.acquire()
        #         for var, var_target in zip(net_list, q_func_vars): 
        #             op = tf.assign(var_target, var)
        #             sess.run(op)
        #         # for i in range(0,len(update_qfunc_op)):
        #         #     sess.run(update_qfunc_op[i])
        #         # for next_var_target in q_func_vars:
        #         #     print(next_var_target.eval(session=sess))
        #         # print(net_list)
        #         net_list_lock.release()  
        #     # print('end_update_qfunc')

        # q_func_vars = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope=tf.get_variable_scope().name + "/q_func")
        # # print(q_func_vars)
        #
        # # 下面update_qfunc还有待商议(U.function),或许用get_session()单独实现一个函数
        # update_qfunc_expr = []
        # net_list_lock.acquire()
        # for var, var_target in zip(in_list, q_func_vars): 
        #     update_qfunc_expr.append(var_target.assign(var))
        # # print(update_target_expr) # 大概就是一系列Assign操作
        #     update_qfunc_expr = tf.group(*update_qfunc_expr)  # tf.group()将语句变为操作？
        # # print(update_target_expr)
        # update_qfunc = U.function([], [], updates=[update_qfunc_expr])

        # q_values = U.function([obs_t_input], q_t)

        return act_f, update_qfunc  # , {'q_values': q_values}






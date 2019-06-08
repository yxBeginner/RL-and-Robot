import gc
from deepq.build_graph import build_act, build_act_with_param_noise
import tensorflow as tf
import baselines.common.tf_util as U


def build_train(make_obs_ph, q_func, num_actions, optimizer, grad_norm_clipping=None, gamma=1.0,
    double_q=True, scope="deepq", reuse=None, param_noise=False, param_noise_filter_func=None):
    """Creates the train function:

    Parameters
    ----------
    make_obs_ph: str -> tf.placeholder or TfInput
        a function that takes a name and creates a placeholder of input with that name
    q_func: (tf.Variable, int, str, bool) -> tf.Variable
        the model that takes the following inputs:
            observation_in: object
                the output of observation placeholder
            num_actions: int
                number of actions
            scope: str
            reuse: bool
                should be passed to outer variable scope
        and returns a tensor of shape (batch_size, num_actions) with values of every action.
    num_actions: int
        number of actions
    reuse: bool
        whether or not to reuse the graph variables
    optimizer: tf.train.Optimizer
        optimizer to use for the Q-learning objective.
    grad_norm_clipping: float or None
        clip gradient norms to this value. If None no clipping is performed.
    gamma: float
        discount rate.
    double_q: bool
        if true will use Double Q Learning (https://arxiv.org/abs/1509.06461).
        In general it is a good idea to keep it enabled.
    scope: str or VariableScope
        optional scope for variable_scope.
    reuse: bool or None
        whether or not the variables should be reused. To be able to reuse the scope must be given.
    param_noise: bool
        whether or not to use parameter space noise (https://arxiv.org/abs/1706.01905)
    param_noise_filter_func: tf.Variable -> bool
        function that decides whether or not a variable should be perturbed. Only applicable
        if param_noise is True. If set to None, default_param_noise_filter is used by default.

    Returns
    -------
    act: (tf.Variable, bool, float) -> tf.Variable
        function to select and action given observation.
`       See the top of the file for details.
    train: (object, np.array, np.array, object, np.array, np.array) -> np.array
        optimize the error in Bellman's equation.
`       See the top of the file for details.
    update_target: () -> ()
        copy the parameters from optimized Q function to the target Q function.
`       See the top of the file for details.
    debug: {str: function}
        a bunch of functions to print debug data like q_values.
    """
    if param_noise:
        act_f = build_act_with_param_noise(make_obs_ph, q_func, num_actions, scope=scope, reuse=reuse,
            param_noise_filter_func=param_noise_filter_func)
    else:
        act_f = build_act(make_obs_ph, q_func, num_actions, scope=scope, reuse=reuse)

    with tf.variable_scope(scope, reuse=reuse):
        multi_step_num = 3  # multi step return 10, 5
        gamma = 0.7  # 折扣率
        # set up placeholders
        obs_t_input = make_obs_ph("obs_t")
        act_t_ph = tf.placeholder(tf.int32, [None], name="action")
        rew_t_ph = tf.placeholder(tf.float32, [None], name="reward")  # 如果要使用长期回报,这里需要一个数组
        obs_tp1_input = make_obs_ph("obs_tp1")
        done_mask_ph = tf.placeholder(tf.float32, [None], name="done")
        importance_weights_ph = tf.placeholder(tf.float32, [None], name="weight")

        # 创建Q network 与 target Q network , 返回所有action 的q值(q_func()) , q_t是一个列表
        # q network evaluation
        q_t = q_func(obs_t_input.get(), num_actions, scope="q_func", reuse=True)  # reuse parameters from act
        q_func_vars = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope=tf.get_variable_scope().name + "/q_func")
        q_tp1 = q_func(obs_tp1_input.get(), num_actions, scope="target_q_func")
        target_q_func_vars = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope=tf.get_variable_scope().name + "/target_q_func")
        # q scores for actions which we know were selected in the given state.
        q_t_selected = tf.reduce_sum(q_t * tf.one_hot(act_t_ph, num_actions), 1)

        # compute estimate of best possible value starting from state at t + 1
        if double_q:
            q_tp1_using_online_net = q_func(obs_tp1_input.get(), num_actions, scope="q_func", reuse=True)
            q_tp1_best_using_online_net = tf.argmax(q_tp1_using_online_net, 1)
            q_tp1_best = tf.reduce_sum(q_tp1 * tf.one_hot(q_tp1_best_using_online_net, num_actions), 1)
        else:
            q_tp1_best = tf.reduce_max(q_tp1, 1)
        q_tp1_best_masked = (1.0 - done_mask_ph) * q_tp1_best

        # compute RHS of bellman equation ,TD目标
        # q_t_selected_target = rew_t_ph + gamma * q_tp1_best_masked
        q_t_selected_target = rew_t_ph + (gamma**multi_step_num) * q_tp1_best_masked  # multi step return

        # compute the error (potentially clipped)
        td_error = q_t_selected - tf.stop_gradient(q_t_selected_target)
        errors = U.huber_loss(td_error)
        weighted_error = tf.reduce_mean(importance_weights_ph * errors)

        # # start cpu
        # with tf.device('/cpu:0'):
        #     # compute optimization op (potentially with gradient clipping)
        #     if grad_norm_clipping is not None:
        #         gradients = optimizer.compute_gradients(weighted_error, var_list=q_func_vars)
        #         for i, (grad, var) in enumerate(gradients):
        #             if grad is not None:
        #                 gradients[i] = (tf.clip_by_norm(grad, grad_norm_clipping), var)
        #         optimize_expr = optimizer.apply_gradients(gradients)
        #     else:
        #         optimize_expr = optimizer.minimize(weighted_error, var_list=q_func_vars)
        # # end cpu
        # compute optimization op (potentially with gradient clipping)
        if grad_norm_clipping is not None:
            gradients = optimizer.compute_gradients(weighted_error, var_list=q_func_vars)
            for i, (grad, var) in enumerate(gradients):
                if grad is not None:
                    gradients[i] = (tf.clip_by_norm(grad, grad_norm_clipping), var)
            optimize_expr = optimizer.apply_gradients(gradients)
        else:
            optimize_expr = optimizer.minimize(weighted_error, var_list=q_func_vars)

        # update_target_fn will be called periodically to copy Q network to target Q network
        # sorted() 不会改变原来的可迭代对象
        update_target_expr = []
        for var, var_target in zip(sorted(q_func_vars, key=lambda v: v.name),  # 利用q_func_vars.name 进行排序,
                                   sorted(target_q_func_vars, key=lambda v: v.name)):
            # print(var)  # 这里var\var_target 就是一个tensor
            # print(var_target)
            update_target_expr.append(var_target.assign(var))
        # print(update_target_expr) # 大概就是一系列Assign操作
        update_target_expr = tf.group(*update_target_expr)  # tf.group()将语句变为操作？

        # 因为是单进程写,多进程读,所以将两种操作分别应用于不同内存区域上,降低lock竞争,仍然有一些不够合理的地方
        # 初始化actor的q网络
        def init_actor_qfunc(sess, net_list):
            # 需要tf.variable_scope(scope, reuse=reuse):　因而写在这里
            # 或使用tf.get_default_session()(不可用上下文管理器)
            with sess.as_default():
                # net_list_lock.acquire()
                # 清空list
                i = len(net_list)
                while i > 0:
                    i -= 1
                    del net_list[i]
                for var_actor in q_func_vars:  # 整体顺序是否正确,有待进一步观察
                    net_list.append(var_actor.eval(session=sess))  # list形式
                # for var_actor in q_func_vars:  # net_list 长度为q_func_vars两倍
                #     net_list.append(var_actor.eval(session=sess))
                gc.collect()  # 释放内存, python3.5 应该不需要
                # net_list_lock.release()  # 释放锁

        len_q_func = len(q_func_vars)

        # 更新actor的q网络
        def update_actor_qfunc(sess, net_list, net_list_lock):
            with sess.as_default():
                net_list_lock.acquire()
                for i_tensor in range(len_q_func):
                    net_list[i_tensor] = q_func_vars[i_tensor].eval(session=sess)
                net_list_lock.release()  # 释放锁

        # 下面三个function分别为整合 train、 update_target 、 q_values
        # Create callable functions
        train = U.function(
            inputs=[
                obs_t_input,
                act_t_ph,
                rew_t_ph,
                obs_tp1_input,
                done_mask_ph,
                importance_weights_ph
            ],
            outputs=td_error,
            updates=[optimize_expr]
        )
        # update_target操作没有输入输出
        update_target = U.function([], [], updates=[update_target_expr])

        q_values = U.function([obs_t_input], q_t)

        return act_f, train, update_target, init_actor_qfunc, update_actor_qfunc, {'q_values': q_values}




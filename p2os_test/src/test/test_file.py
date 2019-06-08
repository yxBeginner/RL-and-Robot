# from memory_profiler import profile
# from memory_profiler import LogFile
# import sys
#
#
# # @profile
# def my_func():
#     a = [1] * (10 ** 6)
#     b = [2] * (2 * 10 ** 7)
#     del b
#     sys.stdout = LogFile('memory_profile_log')
#     return a
#
#
# if __name__ == '__main__':
#     my_func()
#

enable_correction = True


def correct_coordinate_func():
    if enable_correction:
        def this_func(input_value):
            return abs(input_value)
    else:
        def this_func(input_value):
            return input_value
    return this_func


if __name__ == '__main__':
    func = correct_coordinate_func()
    print(func(100))

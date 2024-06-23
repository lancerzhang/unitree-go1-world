import signal


# 定义超时异常
class TimeoutException(Exception):
    pass


# 定义超时处理函数
def timeout_handler(signum, frame):
    raise TimeoutException("Step execution time exceeded 40ms")


# 装饰器函数，限制函数执行时间
def timeout(limit):
    def decorator(func):
        def wrapper(*args, **kwargs):
            signal.signal(signal.SIGALRM, timeout_handler)
            signal.setitimer(signal.ITIMER_REAL, limit)
            try:
                result = func(*args, **kwargs)
            except TimeoutException:
                result = None
            finally:
                signal.setitimer(signal.ITIMER_REAL, 0)  # 关闭闹钟
            return result

        return wrapper

    return decorator

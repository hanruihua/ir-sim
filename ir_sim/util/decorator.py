import time




# calculate the run time of a function
def time_it(name='Function'):
    def decorator(func):
        def wrapper(*args, **kwargs):
            wrapper.count += 1  
            start = time.time() 
            result = func(*args, **kwargs)  
            end = time.time()  
            wrapper.func_count += 1 
            print(f"{name} took {(end - start):.6f} seconds to execute") 
            return result
        wrapper.count = 0  
        wrapper.func_count = 0 
        return wrapper
    return decorator
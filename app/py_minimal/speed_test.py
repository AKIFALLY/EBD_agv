import timeit

array = list(range(100000))  # 測試 10 萬個數字

# 方法 1：使用 map()
def method_map():
    return " ".join(map(str, array))

# 方法 2：使用列表推導式
def method_list_comprehension():
    return " ".join([str(x) for x in array])

# 測試運行時間
time_map = timeit.timeit(method_map, number=10)#測10次總合(秒)
time_list = timeit.timeit(method_list_comprehension, number=10)#測10次總合(秒)

print(f"map() 方法耗時: {time_map*1000:.2f} ms")
print(f"列表推導式耗時: {time_list*1000:.2f} ms")
print(f"map()比列表推導式 慢: {((time_map - time_list)/time_list)*100:.2f} %")

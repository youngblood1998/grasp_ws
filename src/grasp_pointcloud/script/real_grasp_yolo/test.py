# for i in range(10, -1, -1):
#     print(i)

#-------------------------------------

# import numpy as np

# arr = np.array([])
# print(np.mean(arr))

#------------------------------------------------------------------
# def sort_func(a, b):
#     if a[-1]>b[-1]:
#         return -1
#     elif a[-1]<b[-1]:
#         return 1
#     else:
#         return 0

# strawberry_arr = [[0,0,0,0,2], [0,0,0,0,5], [0,0,0,0,1]]
# strawberry_arr.sort(sort_func)
# print(strawberry_arr)

#---------------------------------------------------------------------
# print(False or True)
# a = []
# b = []
# a.extend(b)
# print(len(a))
#----------------------------------------------------------
# a = [5, 6, 1, 3]
# a.sort()
# print(a)
#-----------------------------------------------
# import colorsys
# import random
 
# def get_n_hls_colors(num):
#     hls_colors = []
#     i = 0
#     step = 360.0 / num
#     while i < 360:
#         h = i
#         s = 90 + random.random() * 10
#         l = 50 + random.random() * 10
#         _hlsc = [h / 360.0, l / 100.0, s / 100.0]
#         hls_colors.append(_hlsc)
#         i += step
 
#     return hls_colors

# def ncolors(num):
#     rgb_colors = []
#     if num < 1:
#         return rgb_colors
#     hls_colors = get_n_hls_colors(num)
#     for hlsc in hls_colors:
#         _r, _g, _b = colorsys.hls_to_rgb(hlsc[0], hlsc[1], hlsc[2])
#         r, g, b = [int(x * 255.0) for x in (_r, _g, _b)]
#         rgb_colors.append([r, g, b])
 
#     return rgb_colors

# for i in range(10):
#     print(ncolors(10))
# ---------------------------------------------------
arr = [0,1,2,3,4,5,6,7,8,9]
for i in arr:
    print(i)
    del arr[0]
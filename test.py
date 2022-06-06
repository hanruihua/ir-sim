class Bird:
    #鸟有翅膀
    def isWing(self):
        print("鸟有翅膀")
    #鸟会飞
    def fly(self):
        print("鸟会飞")
class Ostrich(Bird):
    # 重写Bird类的fly()方法
    def fly(self, a):
        print("鸵鸟不会飞")
        print(a)
# 创建Ostrich对象
ostrich = Ostrich()
#调用 Bird 类中的 fly() 方法
ostrich.fly(1)
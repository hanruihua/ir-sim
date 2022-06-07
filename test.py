class Car():
    wheels = None
    def __init__(self, price, color):
        print(120)
        self.price = price
        self.color = color
        self.w = Car.wheels

class Car_s(Car):
    wheels = 4
    def __init__(self, price, color):
        super(Car_s, self).__init__(price, color)
        
        self.ww = Car_s.wheels

car_1 = Car_s(15000, "red")
car_2 = Car_s(15030, "green")

# car_1.wheels = 10
car_2.wheels = 11
# Car.wheels = 1
Car.wheels = 1

print(car_1.wheels)
print(car_2.wheels)
print(car_2.w)
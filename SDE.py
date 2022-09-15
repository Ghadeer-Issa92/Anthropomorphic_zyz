# import re


# def func(a): a = set(a); return a
# a = 'innopolis'
# result = func(a)
# a, result, id(a) == id(result)
# print('a ==', a )
# print('result ==', result)
# print(id(a) == id(result))

# class A:
#     i = 0.5
#     def __init__(self, n):
#         self.data = n

# a = A('innopolis')
# A.var1 = 'software'
# a.name = '007'
# print(A.name)


# Python program to demonstrate
# single inheritance

# Base class
class Parent:
	def func1(self):
		print("This function is in parent class.")

# Derived class


class Child(Parent):
	def func2(self):
		print("This function is in child class.")


# Driver's code
object = Child()
object.func1()
object.func2()


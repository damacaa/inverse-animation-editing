import random


class Task:
    def __init__(self, name, cost):
        self.name = name
        self.cost = cost


class Person:
    def __init__(self, name):
        self.name = name
        self.tasks = []
        self.totalCost = 0

    def addTask(self, task):
        self.tasks.append(task)
        self.totalCost += task.cost


def Distribute(people, tasks):

    random.shuffle(people)
    random.shuffle(tasks)

    tasks.sort(key=lambda x: x.cost, reverse=False)

    id = 0
    for task in tasks:
        person = people[id]
        person.addTask(task)
        id = (id + 1) % len(people)

    '''for task in tasks:
        person = people[random.randint(0, nPeople-1)]
        person.addTask(task)'''

    return people, tasks


nTasks, nPeople = map(int, input().split())

tasks = []
for task in range(nTasks):
    a = list(map(str, input().split()))
    tasks.append(Task(a[0], int(a[1])))

people = []
for person in range(nPeople):
    name = input()
    people.append(Person(name))

people, tasks = Distribute(people, tasks)

for person in people:
    print("___", person.name, "___", person.totalCost)
    for task in person.tasks:
        print(task.name)

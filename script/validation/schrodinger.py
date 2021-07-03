from copy import deepcopy

data = {}
aggregated_data = {}

with open('../../log/rabbits_tests.log', 'r') as file:
    next(file)
    headers = file.readline().strip().split(';')
    headers = headers[0].split(':')[1:] + headers[1:]
    for h in headers: data[h] = []

    aggregated_data = deepcopy(data)
    aggregated_data.pop('worker', None)
    aggregated_data.pop('iteration', None)

    for line in file:
        values = line.strip().split(';')
        values = [float(i) for i in values[0].split(':') + values[1:]]
        for key, value in zip(data.keys(), values): data[key].append(value)

current_iter = 0.0
for i in range(len(data['iteration'])):
    if data['iteration'][i] != current_iter:
        current_iter = current_iter + 1.0
        for key in aggregated_data.keys(): aggregated_data[key].append(0.0)

    for key in aggregated_data.keys(): aggregated_data[key][-1] = aggregated_data[key][-1] + data[key][i]

valid = True
for i in range(1, len(aggregated_data['rabbitCount'])):
    rabbits_killed = aggregated_data['consumedRabbitCount'][i]
    delta = aggregated_data['rabbitCount'][i] - aggregated_data['rabbitReproductionsCount'][i] + aggregated_data['rabbitDeaths'][i] - aggregated_data['rabbitCount'][i-1]
    if rabbits_killed != abs(delta):
        valid = False
        print("rabbits killed: " + str(rabbits_killed))
        print("rabbits delta excluding breeding and starvation: " + str(abs(delta)))

if valid: print("Simulation is valid")
else: print("Simulation is not valid")

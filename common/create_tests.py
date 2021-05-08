import os

def main(filename):
    tasks_file = open(filename)
    map_name = tasks_file.readline()[:-1]
    tasks = [task.split() for task in tasks_file]
    tasks_file.close()
    tasks = [[task[5], task[4], task[7], task[6], float(task[8])] for task in tasks]
    tasks = sorted(tasks, key=lambda x: x[4])
    choosen_tasks = []
    steps = 0
    for ind, task in enumerate(tasks):
        if task[4] > 20:
            if ind % 2 == 0:
                steps += 1
                task[4] = str(task[4])
                choosen_tasks.append(task)
                if steps == 400:
                    break

    tasks_file = open(filename, 'w')
    tasks_file.write(map_name + '\n')
    for task in choosen_tasks:
        tasks_file.write(' '.join(task) + '\n')
    tasks_file.close()

if __name__ == '__main__':
    main(os.path.join('..','maps','lak303d.map.scen'))
    main(os.path.join('..', 'maps', 'hrt201d.map.scen'))
    main(os.path.join('..', 'maps', 'brc505d.map.scen'))
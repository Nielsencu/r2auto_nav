from concurrent.futures import ThreadPoolExecutor
import os

def run_io_tasks_in_parallel(tasks):
    with ThreadPoolExecutor() as executor:
        running_tasks = [executor.submit(task) for task in tasks]
        for running_task in running_tasks:
            running_task.result()
def hello():
    run_io_tasks_in_parallel([
        lambda: os.system("ros2 run py_pubsub talker"),
        lambda: os.system("echo hello"),
    ])

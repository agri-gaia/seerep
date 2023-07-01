import subprocess
res = subprocess.run(["catkin", "locate", "-d"], capture_output=True)
print(res.stdout.decode().replace("\n", ""))

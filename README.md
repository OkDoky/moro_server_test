# moro_server_test


#### How to install (using docker image)

clone source && make docker images
``` 
git clone https://github/OkDoky/moro_server_test.git -b master
cd ~/moro_server_test/Docker
docker build -t moro:latest .
```

after make docker image, make container && run container background
```
docker run --rm --net=host --name "moro_background" moro:latest bash -it -c "roslaunch moro_server_test test.launch"
```

if you want to run without auto start command, and start/stop program manually
```
docker run -it -d --restart=always --net=host --name "moro" moro:latest /bin/bash
docker exec -it moro /bin/bash
roslaunch moro_server_test test.launch
## when you terminate program ctrl+c
## when you start program, command is "roslaunch moro_server_test test.launch"
```

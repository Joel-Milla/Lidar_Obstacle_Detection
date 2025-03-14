# mount current files, map ssh to port 222
docker run --mount type=bind,src=./,dst=/app \
  -d -p 2222:22 --name pcl_env pcl_env

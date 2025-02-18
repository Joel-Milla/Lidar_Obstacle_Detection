docker build -t pcl_env .
docker run -d -p 2222:22 --name pcl_env pcl_env
ssh -X root@localhost -p 2222

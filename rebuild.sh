# 如果没有build文件夹, 新建一个
if [ ! -d "build" ]; then
    mkdir build
fi

cd build && cmake .. && make -j6

rm /root/work_space/tests/my_own_vo/log/vo.txt
./all_tests

cd ..
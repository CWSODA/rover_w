CXX = clang++
CXX_FLAGS = -I ../test_lidar.hpp -std=c++20

./lidar: ../main.cpp ../test_lidar.cpp ../test_lidar.hpp
	$(CXX) $(CXX_FLAGS) ../main.cpp ../lidar.cpp -o lidar

run: ./lidar
	./lidar
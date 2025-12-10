CXX = clang++
CXX_FLAGS = -I ../lidar.hpp -std=c++20

./lidar: ../main.cpp ../lidar.cpp ../lidar.hpp
	$(CXX) $(CXX_FLAGS) ../main.cpp ../lidar.cpp -o lidar

run: ./lidar
	./lidar
OUT= TIPE
CXX= g++
CLIBS= -lglfw -lGL -lX11 -lpthread -lXrandr -lXi -ldl
CFLAGS= -O3 -Wall
OBJECTS= main.o Vector3d.o glad.o
DEPS= Vector3d.hpp

.PHONY= all clean

all: $(OBJECTS)
	$(CXX) $^ -o $(OUT) $(CLIBS)

%.o: %.cpp $(DEPS)
	$(CXX) -c $< -o $@ $(CFLAGS)

clean:
	rm -f $(OUT) $(OBJECTS)


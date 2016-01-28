#sigverse header
SIG_SRC  = $(SIGVERSE_PATH)/include

#オブジェクトファイルの指定
OBJS     = cleanUpRobot.so cleanUpRobot2.so cleanUpUser.so cleanUpUser2.so Avatar.so TrashBox.so Table.so Moderator.so Kinect_Service.so

all: $(OBJS)

#compile
./%.so: ./%.cpp
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -fPIC -shared -o $@ $<

clean:
	rm ./*.so

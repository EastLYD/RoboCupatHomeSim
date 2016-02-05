#sigverse header
SIG_SRC  = $(SIGVERSE_PATH)/include

#オブジェクトファイルの指定
OBJS     =  Robot.so  Human_avatar.so  TrashBox.so Table.so Moderator.so KinectService.so

all: $(OBJS)

#compile
./%.so: ./%.cpp
	g++ -DCONTROLLER -DNDEBUG -DUSE_ODE -DdDOUBLE -I$(SIG_SRC) -I$(SIG_SRC)/comm/controller -fPIC -shared -o $@ $<

clean:
	rm ./*.so

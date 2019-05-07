HEADERS       = GLWidgets.hpp \
                QtWindow.hpp \
                QtMainWindow.hpp \
                logo.h
SOURCES       = GLWidget.cpp \
                main.cpp \
                QtWindow.cpp \
                QtMainWindow.cpp \
                logo.cpp

QT           += widgets

# install
target.path = $$[QT_INSTALL_EXAMPLES]/opengl/hellogl2
INSTALLS += target
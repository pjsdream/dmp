#include <QApplication>
#include <QOpenGLWindow>

int main(int argc, char** argv)
{
  setbuf(stdin, NULL);
  setbuf(stdout, NULL);

  QApplication app(argc, argv);

  QSurfaceFormat format;
  format.setDepthBufferSize(24);
  format.setStencilBufferSize(8);
  //format.setSamples(4);
  format.setVersion(4, 3);
  format.setProfile(QSurfaceFormat::CoreProfile);
  QSurfaceFormat::setDefaultFormat(format);

  QOpenGLWindow* widget = new QOpenGLWindow();

  // showing renderer
  widget->resize(800, 600);
  widget->show();

  app.exec();
  return 0;
}

#include "marble/GeoDataCoordinates.h"
#include "marble/GeoDataDocument.h"
#include "marble/GeoDataPlacemark.h"
#include "marble/GeoDataTreeModel.h"
#include "marble/MarbleModel.h"
#include "marble/MarbleWidget.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include <QApplication>
#include <QObject>
#include <QThread>
#include <QVBoxLayout>
#include <QWidget>
#include <qboxlayout.h>
#include <qobjectdefs.h>
#include <ros/ros.h>
#include <string>
#include <wing_navigator/GLOBAL_POSITION_INT.h>

// TODO: There is a bug, the node does not stop by Ctrl-C. Solve it!

class gps_subscriber : public QObject {
  Q_OBJECT
public:
  gps_subscriber(ros::NodeHandle *nh);

Q_SIGNALS:
  void coordinatesChanged(Marble::GeoDataCoordinates coord, std::string sender);

public Q_SLOTS:
  void start_spinning();

private:
  ros::Subscriber wing_sub;
  ros::Subscriber target_sub;
  void gps_sub_callback(const wing_navigator::GLOBAL_POSITION_INT &msg);
  void tg_gps_sub_callback(const wing_navigator::GLOBAL_POSITION_INT &msg);
  Marble::GeoDataCoordinates
  msg_to_coord(const wing_navigator::GLOBAL_POSITION_INT &msg);
};

gps_subscriber::gps_subscriber(ros::NodeHandle *nh) : QObject() {
  wing_sub = nh->subscribe("/wing_gps_topic_gcs", 1,
                           &gps_subscriber::gps_sub_callback, this);
  target_sub = nh->subscribe("/target_gps_topic", 1,
                             &gps_subscriber::tg_gps_sub_callback, this);
}

void gps_subscriber::start_spinning() { ros::spin(); }

Marble::GeoDataCoordinates
gps_subscriber::msg_to_coord(const wing_navigator::GLOBAL_POSITION_INT &msg) {
  qreal lat = msg.gps_data.latitude;
  qreal lon = msg.gps_data.longitude;

  Marble::GeoDataCoordinates coord(lon, lat, 0.0,
                                   Marble::GeoDataCoordinates::Degree);
  return coord;
}

void gps_subscriber::gps_sub_callback(
    const wing_navigator::GLOBAL_POSITION_INT &msg) {
  Marble::GeoDataCoordinates coord = msg_to_coord(msg);
  emit coordinatesChanged(coord, "wing");
}
void gps_subscriber::tg_gps_sub_callback(
    const wing_navigator::GLOBAL_POSITION_INT &msg) {
  Marble::GeoDataCoordinates coord = msg_to_coord(msg);
  emit coordinatesChanged(coord, "target");
}

// Window Class
class Window : public QWidget {
  Q_OBJECT
public:
  explicit Window(const ros::NodeHandle &nh, QWidget *parent = nullptr);
  void start_subscribing();

public Q_SLOTS:
  // updates the position of marker
  void setNewCoordinates(const Marble::GeoDataCoordinates &coord,
                         const std::string &sender);

private:
  Marble::MarbleWidget *m_marbleWidget;
  gps_subscriber *m_worker;
  ros::NodeHandle _nh;
  Marble::GeoDataPlacemark *m_wing;
  Marble::GeoDataPlacemark *m_target;
  QThread *m_thread;
};

Window::Window(const ros::NodeHandle &nh, QWidget *parent)
    : QWidget(parent), m_marbleWidget(new Marble::MarbleWidget), _nh(nh) {
  QVBoxLayout *layout = new QVBoxLayout(this);
  layout->addWidget(m_marbleWidget);
  setLayout(layout);

  // Load the OpenStreetMap map
  m_marbleWidget->setMapThemeId(
      QStringLiteral("earth/vectorosm/vectorosm.dgml"));
  m_marbleWidget->setProjection(Marble::Spherical);
  setGeometry(80, 60, 1000, 800);
  Marble::GeoDataCoordinates test_location(35.74743520638767, 51.60374888014409,
                                           0.0,
                                           Marble::GeoDataCoordinates::Degree);
  m_marbleWidget->centerOn(test_location);
  m_marbleWidget->setZoom(2300);

  m_wing = new Marble::GeoDataPlacemark(QStringLiteral("wing"));
  m_target = new Marble::GeoDataPlacemark(QStringLiteral("target"));

  Marble::GeoDataDocument *document = new Marble::GeoDataDocument;

  document->append(m_wing);
  document->append(m_target);

  m_marbleWidget->model()->treeModel()->addDocument(document);

  show();
}

void Window::start_subscribing() {

  // funnywing placemark
  m_thread = new QThread;
  m_worker = new gps_subscriber(&_nh);
  m_worker->moveToThread(m_thread);

  connect(m_worker,
          SIGNAL(coordinatesChanged(Marble::GeoDataCoordinates, std::string)),
          this, SLOT(setNewCoordinates(Marble::GeoDataCoordinates, std::string)),
          Qt::BlockingQueuedConnection);

  connect(m_thread, SIGNAL(started()), m_worker, SLOT(start_spinning()));

  m_thread->start();
}

void Window::setNewCoordinates(const Marble::GeoDataCoordinates &coord,
                               const std::string &sender) {

  // commented because we use the same object for both markers and I think this
  // gives one thing to us. so we used a parameter in signal for specification
  // of sender. gps_subscriber *worker = qobject_cast<gps_subscriber
  // *>(sender());

  if (sender == "wing") {
    m_wing->setCoordinate(coord);
    m_marbleWidget->model()->treeModel()->updateFeature(m_wing);
  } else if (sender == "target") {
    m_target->setCoordinate(coord);
    m_marbleWidget->model()->treeModel()->updateFeature(m_target);
  }
}

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  ros::init(argc, argv, "simple_marble_map");
  ros::NodeHandle nh;

  Window window(nh);
  window.start_subscribing();

  return app.exec();
};
#include "simple_marble_map.moc"

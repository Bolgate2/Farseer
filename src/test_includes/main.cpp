// package includes
#include <iostream>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <uuid_v4/uuid_v4.h>
// qt includes
#include <QApplication>
#include <QWidget>
#include <QLabel>
#include <QString>
#include <QVBoxLayout>

// converts an Eigen vector or matrix to a string
template<typename T>
static std::string toString(const Eigen::DenseBase<T>& mat){
    std::stringstream ss;
    ss << mat;
    return ss.str();
}


int main(int argc, char **argv)
{
    std::cout << "Hello World!" << std::endl;
    // testing imports
    // Eigen
    Eigen::Vector3i vec {1,2,3};
    std::string eigenStr = toString(vec.transpose());

    // uuid
    UUIDv4::UUIDGenerator<std::mt19937_64> uuidGenerator;
    UUIDv4::UUID uuid = uuidGenerator.getUUID();
    std::string uuidStr = uuid.str();
    
    // json
    nlohmann::json jason = {
        {"name", "jeff"},
        {"jump street", 22}
    };
    std::string jsonStr = jason.dump(2);

    // creating qt app
    QApplication app (argc, argv);
    // creating top level app and adding layout
    QWidget* topWidget = new QWidget;
    QVBoxLayout* layout = new QVBoxLayout(topWidget);
    // creating a label for each string
    QLabel* eigenLabel = new QLabel(QString::fromStdString("Eigen output:\n" + eigenStr));
    QLabel* uuidLabel = new QLabel(QString::fromStdString("uuid output:\n" + uuidStr));
    QLabel* jsonLabel = new QLabel(QString::fromStdString("json output:\n" + jsonStr));

    // adding labels to widget
    layout->addWidget(eigenLabel);
    layout->addWidget(uuidLabel);
    layout->addWidget(jsonLabel);

    topWidget->show();

    return app.exec();
}
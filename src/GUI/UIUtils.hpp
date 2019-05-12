#ifndef GUI_UIUTILS_HPP
#define GUI_UIUTILS_HPP

#include <QtCore/QString>
#include "../model/KinematicsModel.hpp"


bool isValidJAFormat(const QString& angleInput, double result[NUM_OF_JOINTS]);

#endif
/*
###############################################################################
#                                                                             #
# The MIT License                                                             #
#                                                                             #
# Copyright (C) 2017 by Juergen Skrotzky (JorgenVikingGod@gmail.com)          #
#               >> https://github.com/Jorgen-VikingGod                        #
#                                                                             #
# Sources: https://github.com/Jorgen-VikingGod/Qt-Frameless-Window-DarkStyle  #
#                                                                             #
###############################################################################
*/

#ifndef DarkStyle_HPP
#define DarkStyle_HPP

#include <QApplication>
#include <QFile>
#include <QFont>
#include <QProxyStyle>
#include <QStyleFactory>

class DarkStyle : public QProxyStyle
{
  Q_OBJECT

public:
  DarkStyle();
  explicit DarkStyle(QStyle* style);

  QStyle* baseStyle() const;

  void polish(QPalette& palette) override;
  void polish(QApplication* app) override;

private:
  QStyle* styleBase(QStyle* style = nullptr) const;
};

#endif // DarkStyle_HPP

/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <QColor>
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QToolBar>
#include <QMenu>

#include "myviz.h"
#include <QDebug>


// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QHBoxLayout* QHBox, QWidget* parent )
  : rviz::VisualizationFrame( parent )
  , QHBox_(QHBox)
{
  setSplashPath("C:/dev_ws/src/custom_gui/resources/images/Signature_K_1Row.png");
  initialize("c:/dev_ws/src/stella_remote_pc_n2/stella_navigation/rviz/stella_navigation.rviz");

  setMenuBar( 0 );
  setStatusBar( 0 );
  setHideButtonVisibility(false);
  toolbar_->removeAction(toolbar_->actions()[6]);
  toolbar_->removeAction(toolbar_->actions()[5]);
  toolbar_->removeAction(toolbar_->actions()[4]);
  toolbar_->removeAction(toolbar_->actions()[3]);
  toolbar_->setContextMenuPolicy(Qt::PreventContextMenu);
  toolbar_->setMovable(false);
  manager_->startUpdate();
  activateWindow();
  QHBox_->insertWidget( 0, this );
}

void MyViz::reset_()
{
  reset();
}

// Destructor.
MyViz::~MyViz()
{
  delete QHBox_;
}
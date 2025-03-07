/*
 Copyright (C) 2021 Eric Wasylishen

 This file is part of TrenchBroom.

 TrenchBroom is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 TrenchBroom is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with TrenchBroom. If not, see <http://www.gnu.org/licenses/>.
 */

#include "SignalDelayer.h"

#include <QDebug>
#include <QMetaMethod>
#include <QTimer>

namespace TrenchBroom {
    namespace View {
        SignalDelayer::SignalDelayer(QObject* parent) :
        QObject{parent},
        m_isQueued{false} {}

        void SignalDelayer::queueSignal() {
            static const QMetaMethod processSignalMetaMethod = QMetaMethod::fromSignal(&SignalDelayer::processSignal);
            if (!isSignalConnected(processSignalMetaMethod)) {
                qWarning() << "queueSignal called with nothing connected to processSignal";
            }

            if (m_isQueued) {
                return;
            }

            m_isQueued = true;

            QTimer::singleShot(0, this, [&](){
                m_isQueued = false;

                emit processSignal();
            });
        }
    }
}

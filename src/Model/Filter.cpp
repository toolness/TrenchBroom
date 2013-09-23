/*
 Copyright (C) 2010-2013 Kristian Duske
 
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

#include "Filter.h"

#include "Model/Brush.h"
#include "Model/BrushFace.h"
#include "Model/Entity.h"
#include "Model/EntityProperties.h"

namespace TrenchBroom {
    namespace Model {
        bool Filter::visible(const Entity* entity) const {
            if (entity->worldspawn())
                return false;
            return true;
        }
        
        bool Filter::visible(const Brush* brush) const {
            return true;
        }
        
        bool Filter::visible(const BrushFace* face) const {
            return true;
        }
        
        bool Filter::pickable(const Entity* entity) const {
            if (entity->worldspawn())
                return false;
            return true;
        }
        
        bool Filter::pickable(const Brush* brush) const {
            return true;
        }
        
        bool Filter::pickable(const BrushFace* face) const {
            return true;
        }
    }
}

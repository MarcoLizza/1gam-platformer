--[[

Copyright (c) 2016 by Marco Lizza (marco.lizza@gmail.com)

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgement in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

]]--

-- MODULE INCLUSIONS -----------------------------------------------------------

local config = require('game.config')

-- MODULE DECLARATION ----------------------------------------------------------

local constants = {
}

-- MODULE VARIABLES ------------------------------------------------------------

constants.VERSION = '0.1.0'

constants.IDENTITY = 'Darn!'

constants.SCREEN_WIDTH = 480 -- QHDTV
constants.SCREEN_HEIGHT = 270
constants.SCREEN_RECT = { 0, 0, constants.SCREEN_WIDTH, constants.SCREEN_HEIGHT }
constants.SCREEN_CENTER = { constants.SCREEN_WIDTH / 2, constants.SCREEN_HEIGHT / 2 }

constants.WINDOW_WIDTH = constants.SCREEN_WIDTH * config.display.scale
constants.WINDOW_HEIGHT = constants.SCREEN_HEIGHT * config.display.scale
constants.WINDOW_TITLE = string.format('.: %s :. (%s)', string.upper(constants.IDENTITY), constants.VERSION)

-- END OF MODULE ---------------------------------------------------------------

return constants

-- END OF FILE -----------------------------------------------------------------

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

local constants = require('game.constants')
local config = require('game.config')
local Input = require('lib.input')
local Stateful = require('lib.stateful')

-- LOCAL VARIABLES -------------------------------------------------------------

local _stateful = nil
local _time = 0
local _input = nil
local _canvas = nil

-- LOCAL FUNCTIONS -------------------------------------------------------------

local function auto_size(ratio)
  if config.display.scale == -1 then
    local width, height = love.window.getDesktopDimensions()
    width = width * ratio
    height = height * ratio
    while true do
      local scale = config.display.scale + 1
      local w, h = constants.SCREEN_WIDTH * scale, constants.SCREEN_HEIGHT * scale
      if w > width or h > height then
        -- We don't pass any specific flag since we want to keep the ones we
        -- chose at the beginning.
        love.window.setMode(constants.WINDOW_WIDTH, constants.WINDOW_HEIGHT)
        break
      end
      config.display.scale = scale
      constants.WINDOW_WIDTH = w
      constants.WINDOW_HEIGHT = h
    end
  end
end

-- ENGINE CALLBACKS ------------------------------------------------------------

function love.load(args)
  if args[#args] == '-debug' then require('mobdebug').start() end

  -- Autosize the window, keeping a margin.
  auto_size(config.display.ratio or 0.95)

  -- We stay true to a real "pixelized" feel.
  --
  -- Note that we need need to set the filter before creating the canvas in
  -- order the setting to be used by the canvas itself!
  love.graphics.setDefaultFilter('nearest', 'nearest', 1)

  -- Create the offscreen canvas, so that no scaling mumbo-jumbo need to be
  -- performed during the drawing itself.
  _canvas = love.graphics.newCanvas(constants.SCREEN_WIDTH,
      constants.SCREEN_HEIGHT)

  -- Initializes the input handler.
  _input = Input.new()
  _input:initialize({
        ['z'] = 'action',
        ['x'] = 'action'
      }, {
        ['continuous'] = 0,
        ['action'] = 0.20
      })

  -- Initializes the state-engine.
  _stateful = Stateful.new()
  _stateful:initialize({
        ['splash'] = require('game.states.splash'),
        ['game'] = require('game.states.game')
      }, {
        level = 0
      })
  _stateful:switch_to('game')
end

function love.keypressed(key, scancode, isrepeat)
  if key == 'f10' then
    config.debug.fps = not config.debug.fps
  elseif key == 'f11' then
    config.debug.cheat = not config.debug.cheat
  elseif key == 'f12' then
    local screenshot = love.graphics.newScreenshot()
    screenshot:encode('png', os.time() .. '.png')
  end
end

function love.update(dt)
  -- Update the input handler. Always propagates the keys state, in order to
  -- handle also the "no input" case.
  local keys = _input:update(dt)
  _stateful:input(keys, dt)
  
  -- Update the state manager.
  local start = love.timer.getTime()
  _stateful:update(dt)
  _time = love.timer.getTime() - start
end

function love.draw()
  love.graphics.setCanvas(_canvas)
  love.graphics.clear()
  _stateful:draw()
  if config.debug.fps then
    love.graphics.print(love.timer.getFPS() .. '\n' .. _time, 0, 0)
  end
  love.graphics.setCanvas()

  local scale = config.display.scale
  love.graphics.draw(_canvas, 0, 0, 0, scale, scale)
end

-- END OF FILE -----------------------------------------------------------------

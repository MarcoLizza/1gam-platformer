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

-- MODULE DECLARATION ----------------------------------------------------------

local graphics = {
}

-- LOCAL CONSTANTS -------------------------------------------------------------

local CHARSET = ' !"#$%&\'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~'

local COLORS = {
  ['aliceblue'] = { 240, 248, 255 },
  ['antiquewhite'] = { 250, 235, 215 },
  ['aqua'] = { 0, 255, 255 },
  ['aquamarine'] = { 127, 255, 212 },
  ['azure'] = { 240, 255, 255 },
  ['beige'] = { 245, 245, 220 },
  ['bisque'] = { 255, 228, 196},
  ['black'] = { 0, 0, 0 },
  ['blanchedalmond'] = { 255, 235, 205 },
  ['blue'] = { 0, 0, 255 },
  ['blueviolet'] = { 138, 43, 226 },
  ['brown'] = { 165, 42, 42 },
  ['burlywood'] = { 222, 184, 135 },
  ['cadetblue'] = { 95, 158, 160 },
  ['chartreuse'] = { 127, 255, 0 },
  ['chocolate'] = { 210, 105, 30 },
  ['coral'] = { 255, 127, 80 },
  ['cornflowerblue'] = { 100, 149, 237 },
  ['cornsilk'] = { 255, 248, 220 },
  ['crimson'] = { 220, 20, 60 },
  ['cyan'] = { 0, 255, 255 },
  ['darkblue'] = { 0, 0, 139 },
  ['darkcyan'] = { 0, 139, 139 },
  ['darkgoldenrod'] = { 184, 134, 11 },
  ['darkgray'] = { 169, 169, 169 },
  ['darkgreen'] = { 0, 100, 0 },
  ['darkgrey'] = { 169, 169, 169 },
  ['darkkhaki'] = { 189, 183, 107 },
  ['darkmagenta'] = { 139, 0, 139 },
  ['darkolivegreen'] = { 85, 107, 47 },
  ['darkorange'] = { 255, 140, 0 },
  ['darkorchid'] = { 153, 50, 204 },
  ['darkred'] = { 139, 0, 0 },
  ['darksalmon'] = { 233, 150, 122 },
  ['darkseagreen'] = { 143, 188, 143 },
  ['darkslateblue'] = { 72, 61, 139 },
  ['darkslategray'] = { 47, 79, 79 },
  ['darkslategrey'] = { 47, 79, 79 },
  ['darkturquoise'] = { 0, 206, 209 },
  ['darkviolet'] = { 148, 0, 211 },
  ['deeppink'] = { 255, 20, 147 },
  ['deepskyblue'] = { 0, 191, 255 },
  ['dimgray'] = { 105, 105, 105 },
  ['dimgrey'] = { 105, 105, 105 },
  ['dodgerblue'] = { 30, 144, 255 },
  ['firebrick'] = { 178, 34, 34 },
  ['floralwhite'] = { 255, 250, 240 },
  ['forestgreen'] = { 34, 139, 34 },
  ['fuchsia'] = { 255, 0, 255 },
  ['gainsboro'] = { 220, 220, 220 },
  ['ghostwhite'] = { 248, 248, 255 },
  ['gold'] = { 255, 215, 0 },
  ['goldenrod'] = { 218, 165, 32 },
  ['gray'] = { 128, 128, 128 },
  ['grey'] = { 128, 128, 128 },
  ['green'] = { 0, 128, 0 },
  ['greenyellow'] = { 173, 255, 47 },
  ['honeydew'] = { 240, 255, 240 },
  ['hotpink'] = { 255, 105, 180 },
  ['indianred'] = { 205, 92, 92 },
  ['indigo'] = { 75, 0, 130 },
  ['ivory'] = { 255, 255, 240 },
  ['khaki'] = { 240, 230, 140 },
  ['lavender'] = { 230, 230, 250 },
  ['lavenderblush'] = { 255, 240, 245 },
  ['lawngreen'] = { 124, 252, 0 },
  ['lemonchiffon'] = { 255, 250, 205 },
  ['lightblue'] = { 173, 216, 230 },
  ['lightcoral'] = { 240, 128, 128 },
  ['lightcyan'] = { 224, 255, 255 },
  ['lightgoldenrodyellow'] = { 250, 250, 210 },
  ['lightgray'] = { 211, 211, 211 },
  ['lightgreen'] = { 144, 238, 144 },
  ['lightgrey'] = { 211, 211, 211 },
  ['lightpink'] = { 255, 182, 193 },
  ['lightsalmon'] = { 255, 160, 122 },
  ['lightseagreen'] = { 32, 178, 170 },
  ['lightskyblue'] = { 135, 206, 250 },
  ['lightslategray'] = { 119, 136, 153 },
  ['lightslategrey'] = { 119, 136, 153 },
  ['lightsteelblue'] = { 176, 196, 222 },
  ['lightyellow'] = { 255, 255, 224 },
  ['lime'] = { 0, 255, 0 },
  ['limegreen'] = { 50, 205, 50 },
  ['linen'] = { 250, 240, 230 },
  ['magenta'] = { 255, 0, 255 },
  ['maroon'] = { 128, 0, 0 },
  ['mediumaquamarine'] = { 102, 205, 170 },
  ['mediumblue'] = { 0, 0, 205 },
  ['mediumorchid'] = { 186, 85, 211 },
  ['mediumpurple'] = { 147, 112, 219 },
  ['mediumseagreen'] = { 60, 179, 113 },
  ['mediumslateblue'] = { 123, 104, 238 },
  ['mediumspringgreen'] = { 0, 250, 154 },
  ['mediumturquoise'] = { 72, 209, 204 },
  ['mediumvioletred'] = { 199, 21, 133 },
  ['midnightblue'] = { 25, 25, 112 },
  ['mintcream'] = { 245, 255, 250 },
  ['mistyrose'] = { 255, 228, 225 },
  ['moccasin'] = { 255, 228, 181 },
  ['navajowhite'] = { 255, 222, 173 },
  ['navy'] = { 0, 0, 128 },
  ['oldlace'] = { 253, 245, 230 },
  ['olive'] = { 128, 128, 0 },
  ['olivedrab'] = { 107, 142, 35 },
  ['orange'] = { 255, 165, 0 },
  ['orangered'] = { 255, 69, 0 },
  ['orchid'] = { 218, 112, 214 },
  ['palegoldenrod'] = { 238, 232, 170 },
  ['palegreen'] = { 152, 251, 152 },
  ['paleturquoise'] = { 175, 238, 238 },
  ['palevioletred'] = { 219, 112, 147 },
  ['papayawhip'] = { 255, 239, 213 },
  ['peachpuff'] = { 255, 218, 185 },
  ['peru'] = { 205, 133, 63 },
  ['pink'] = { 255, 192, 203 },
  ['plum'] = { 221, 160, 221 },
  ['powderblue'] = { 176, 224, 230 },
  ['purple'] = { 128, 0, 128 },
  ['red'] = { 255, 0, 0 },
  ['rosybrown'] = { 188, 143, 143 },
  ['royalblue'] = { 65, 105, 225 },
  ['saddlebrown'] = { 139, 69, 19 },
  ['salmon'] = { 250, 128, 114 },
  ['sandybrown'] = { 244, 164, 96 },
  ['seagreen'] = { 46, 139, 87 },
  ['seashell'] = { 255, 245, 238 },
  ['sienna'] = { 160, 82, 45 },
  ['silver'] = { 192, 192, 192 },
  ['skyblue'] = { 135, 206, 235 },
  ['slateblue'] = { 106, 90, 205 },
  ['slategray'] = { 112, 128, 144 },
  ['slategrey'] = { 112, 128, 144 },
  ['snow'] = { 255, 250, 250 },
  ['springgreen'] = { 0, 255, 127 },
  ['steelblue'] = { 70, 130, 180 },
  ['tan'] = { 210, 180, 140 },
  ['teal'] = { 0, 128, 128 },
  ['thistle'] = { 216, 191, 216 },
  ['tomato'] = { 255, 99, 71 },
  ['turquoise'] = { 64, 224, 208 },
  ['violet'] = { 238, 130, 238 },
  ['wheat'] = { 245, 222, 179 },
  ['white'] = { 255, 255, 255 },
  ['whitesmoke'] = { 245, 245, 245 },
  ['yellow'] = { 255, 255, 0 },
  ['yellowgreen'] = { 154, 205, 50 }
}

-- Preload some fonts. We are using the proper (pedantic) syntax to access the
-- map items in this case, to be safe.
local FONTS = {
  ['retro-computer'] = love.graphics.newImageFont('assets/fonts/retro_computer_regular_14.png', CHARSET),
  ['silkscreen'] = love.graphics.newImageFont('assets/fonts/silkscreen_normal_8.png', CHARSET)
}

-- MODULE VARIABLES ------------------------------------------------------------

local _queue = {
}

-- MODULE FUNCTIONS ------------------------------------------------------------

function graphics.enqueue(closure, z_index)
  _queue[#_queue + 1] = {
        closure = closure,
        z_index = z_index or 0
      }
end

function graphics.process()
  table.sort(_queue, function(a, b)
      return a.z_index < b.z_index
     end)
  for _, v in ipairs(_queue) do
    v.closure()
  end
  _queue = {}
end

function graphics.fill(color, alpha)
  color = type(color) == 'table' and color or COLORS[color]
  alpha = alpha or 255

  if alpha == 0 then
    return
  end

  local r, g, b = unpack(color)

  love.graphics.setColor({ r, g, b, alpha })
  love.graphics.rectangle('fill', 0, 0,
      constants.SCREEN_WIDTH, constants.SCREEN_HEIGHT)
end

function graphics.rectangle(x, y, width, height, color, alpha)
  color = type(color) == 'table' and color or COLORS[color]
  alpha = alpha or 255

  if alpha == 0 then
    return
  end

  local r, g, b = unpack(color)
  
  love.graphics.setColor({ r, g, b, alpha })
  love.graphics.rectangle('fill', x, y,
      width, height)
end

function graphics.square(x, y, size, color, alpha)
  color = type(color) == 'table' and color or COLORS[color]
  alpha = alpha or 255

  if alpha == 0 then
    return
  end

  local r, g, b = unpack(color)
  
  love.graphics.setColor({ r, g, b, alpha })
  love.graphics.rectangle('fill', x, y,
      size, size)
end

function graphics.circle(x, y, radius, color, alpha)
  color = type(color) == 'table' and color or COLORS[color]
  alpha = alpha or 255

  if alpha == 0 then
    return
  end

  local r, g, b = unpack(color)

  love.graphics.setColor({ r, g, b, alpha })
  love.graphics.circle('fill', x, y, radius, 100)
end

function graphics.line(x0, y0, x1, y1, color, alpha, width)
  color = type(color) == 'table' and color or COLORS[color]
  alpha = alpha or 255
  width = width or 1

  if alpha == 0 then
    return
  end

  local r, g, b = unpack(color)

  love.graphics.setColor({ r, g, b, alpha })
  love.graphics.setLineWidth(width)
  love.graphics.line({ x0, y0, x1, y1 })
end

function graphics.image(image, x, y, ox, oy, width, height, alpha)
  ox = ox or 0
  oy = oy or 0
  width = width or image:getWidth()
  height = height or image:getHeight()
  alpha = alpha or 255

  if alpha == 0 then
    return
  end

  love.graphics.setColor({ 255, 255, 255, alpha })
  love.graphics.setScissor(x, y, width, height)
  love.graphics.draw(image, x, y, 0, 1, 1, ox, oy)
  love.graphics.setScissor()
end

function graphics.measure(text, face, scale)
  scale = scale or 1

  local font = FONTS[face]

  local width = font:getWidth(text)
  local height = font:getHeight()
  
  return width * scale, height * scale
end

function graphics.text(text, rectangle, face, color, halign, valign, scale, alpha)
  color = type(color) == 'table' and color or COLORS[color]
  alpha = alpha or 255
  halign = halign or 'center'
  valign = valign or 'middle'
  scale = scale or 1

  if alpha == 0 then
    return
  end

  local font = FONTS[face]

  local x, y = rectangle[1], rectangle[2]

  if rectangle[3] and rectangle[4] then -- FIXME: width/height or right/bottom???
    local width = font:getWidth(text) * scale
    local height = font:getHeight() * scale
    if halign == 'right' then
      x = rectangle[3] - width
    elseif halign == 'center' then
      x = (rectangle[3] - rectangle[1] - width) / 2
    end
    if valign == 'bottom' then
      y = rectangle[4] - height
    elseif valign == 'middle' then
      y = (rectangle[4] - rectangle[2] - height) / 2
    end
  end

  local r, g, b = unpack(color)

  love.graphics.push()
  love.graphics.scale(scale)
  love.graphics.setFont(font)
  love.graphics.setColor({ r, g, b, alpha })
  love.graphics.print(text, x / scale, y / scale)
  love.graphics.pop()
end

-- END OF MODULE ---------------------------------------------------------------

return graphics

-- END OF FILE------------------------------------------------------------------

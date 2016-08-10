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

-- https://springrts.com/wiki/Lua_Performance#TEST_3:_Unpack_A_Table

-- MODULE INCLUSIONS -----------------------------------------------------------

-- MODULE DECLARATION ----------------------------------------------------------

local Entities = {
  _VERSION = '0.4.1'
}

-- MODULE OBJECT CONSTRUCTOR ---------------------------------------------------

Entities.__index = Entities

function Entities.new()
  local self = setmetatable({}, Entities)
  return self
end

-- LOCAL CONSTANTS -------------------------------------------------------------

local EPSILON = 1e-10

-- LOCAL FUNCTIONS -------------------------------------------------------------

local function edelta(a, b)
  local delta = a - b
  if delta <= -EPSILON then
    return -1
  elseif delta >= EPSILON then
    return 1
  else
    return 0
  end
end

local function esign(x)
  return edelta(x, 0)
end

-- http://hamaluik.com/posts/simple-aabb-collision-using-minkowski-difference/
-- http://hamaluik.com/posts/swept-aabb-collision-using-minkowski-difference/
local function minkowski(a, b)
  local al, at, ar, ab = unpack(a)
  local bl, bt, br, bb = unpack(b)
  return al - br, at - bb, ar - bl, ab - bt
end

local function cross_product(vx, vy, wx, wy)
  return vx * wy - vy * wx
end

-- Derived from the following code, with the simplication that the
-- first vector origin is <0, 0>.
-- https://github.com/pgkelley4/line-segments-intersect/blob/master/js/line-segments-intersect.js
local function intersect(rx, ry, qx, qy, sx, sy)
  local numerator = cross_product(qx, qy, rx, ry)
  local denominator = cross_product(rx, ry, sx, sy)

  if esign(numerator) == 0 and esign(denominator == 0) then
    -- The lines are colinear, check if they overlap
    -- TODO: calculate intersection point
    return math.huge
  end

  if esign(denominator) == 0 then
    -- The lines are parallel
    return math.huge
  end
    
  local u = numerator / denominator
  local t = cross_product(qx, qy, sx, sy) / denominator
  if esign(t) >= 0 and edelta(t, 1) <= 0 and esign(u) >= 0 and edelta(u, 1) <= 0 then
    return t
  end
  return math.huge
end

local function colliding(a, da, b, db)
  local xl, xt, xr, xb = minkowski(a, b)
  -- If the Minkowsky AABB wraps the origin point around the two entities
  -- are colliding.
  if esign(xl) <= 0 and esign(xr) >= 0 and esign(xt) <= 0 and esign(xb) >= 0 then
    return true, 0
  else
    -- Otherwise, calculate the relative motion between the two AABBs.
    local adx, ady = unpack(da)
    local bdx, bdy = unpack(db)
    local rx, ry = adx - bdx, ady - bdy

    -- Ray-cast the relative motion vector against the Minkowski AABB.
    -- As an optimization (and simplification) we are storing the
    -- starting point and the delta movement.
    local segments = {
        { xl, xt, 0, xb - xt }, -- down
        { xl, xb, xr - xl, 0 }, -- right
        { xr, xb, 0, xt - xb }, -- up
        { xr, xt, xl - xr, 0 }  -- left
      }
    local min_h = math.huge
    for _, segment in ipairs(segments) do
      local qx, qy, sx, sy = unpack(segment)
      local h = intersect(rx, ry, qx, qy, sx, sy);
      if min_h > h then
        min_h = h
      end
    end

    if min_h < math.huge then
      return true, min_h
    else
      return false, 1
    end
  end
end

local function pass(entity, h, normal)
  local ox, oy = unpack(entity.position)
  local dx, dy = unpack(entity.delta)
  
  entity.position = { ox + dx, oy + dy }
end

local function touch(entity, h, normal)
  local ox, oy = unpack(entity.position)
  local dx, dy = unpack(entity.delta)
  
  entity.position = { ox + dx * h, oy + dy * h }
end

local function deflect(entity, h, normal)
  local ox, oy = unpack(entity.position)
  local dx, dy = unpack(entity.delta)
  local nx, ny = unpack(normal)
  local x, y = ox + dx * h, oy + dy * h
  
  local r = 1 - h
  
  if esign(nx) > 0 then
    x = x - dx * r
  end
  if esign(ny) > 0 then
    y = y - dy * r
  end
  
  entity.position = { x, y }
end

local function slide(entity, h, normal)
  local ox, oy = unpack(entity.position)
  local dx, dy = unpack(entity.delta)
  local nx, ny = unpack(normal)
  local x, y = ox + dx * h, oy + dy * h
  
  local r = 1 - h
  
  local dot_product = (dx * ny + dy * nx) * r
  
  entity.position = { x + dot_product * ny, y + dot_product * nx }
end

local function hash(x, y)
  local id = string.format('%d@%d', x, y)
  return id
end

-- MODULE FUNCTIONS ------------------------------------------------------------

function Entities:initialize(grid_size)
  -- Store the (optional) grid size.
  self.grid_size = grid_size or 65536

  self:reset()
end

function Entities:reset()
  self.active = {}
  self.incoming = {}
end

function Entities:update(dt, comparator, filter)
  -- If there are any waiting recently added entities, we merge them in the
  -- active entities list. The active list is kept sorted, if a proper
  -- comparator was provided.
  if #self.incoming > 0 then
    for _, entity in ipairs(self.incoming) do
      table.insert(self.active, entity);
    end
    self.incoming = {}
    if comparator then
      table.sort(self.active, comparator)
    end
  end

  -- Update and keep track of the entities that need to be removed.
  --
  -- Since we need to keep the entities relative sorting, we remove "dead"
  -- entities from the back to front. To achive this we "push" the
  -- indices at the front of the to-be-removed list. That way, when
  -- we traverse it we can safely remove the elements as we go.
  --
  -- We are going to calulate and store the delta movement vectors for
  -- each entity (using the [delta] property). They will come handy when
  -- applying collision detection.
  local zombies = {}
  for index, entity in ipairs(self.active) do
    local ox, oy = unpack(entity.position)
    entity:update(dt) -- TODO: should update() return the new position? Doubtful...
    local nx, ny = unpack(entity.position)
    entity.position = { ox, oy }
    entity.delta = { nx - ox, ny - oy }

    if entity.is_alive and not entity:is_alive() then
      table.insert(zombies, 1, index);
    end
  end
  for _, index in ipairs(zombies) do
    table.remove(self.active, index)
  end
  
  -- Rebuild the spatial-hashmap if needed.
  if filter then
    local buckets = self:partition(self.active, self.grid_size)
    for _, entities in pairs(buckets) do
      self:resolve(entities, filter)
    end
  end
end

function Entities:draw()
  for _, entity in pairs(self.active) do
    entity:draw()
  end
end

function Entities:push(entity)
  -- We store thre entity-manager reference in the entity itself. Could be
  -- useful.
  entity.entities = self

  -- We enqueue the added entries in a temporary list. Then, in the "update"
  -- function we merge the entries with the active entries list and sort it.
  -- 
  -- The rationale for this is to decouple the entities scan/iteration and
  -- the addition. For example, it could happen that during an iteration we
  -- add new entities; however we cannot modify the active entities list
  -- content while we iterate.
  --
  -- We are using the "table" namespace functions since we are possibly
  -- continously scrambling the content by reordering it.
  table.insert(self.incoming, entity)
end

function Entities:find(filter)
  for _, entity in ipairs(self.active) do
    if filter(entity) then
      return entity
    end
  end
  for _, entity in ipairs(self.incoming) do
    if filter(entity) then
      return entity
    end
  end
  return nil
end

function Entities:partition(entities, size)
  local buckets = {}

  for _, entity in ipairs(entities) do
    -- If the entity does not have the [aabb] method we consider it to be
    -- "ephemeral" in nature (e.g. sparkles, smoke, bubbles, etc...). It will
    -- be ignored and won't count toward collision.
    if entity.aabb then
      -- We find the belonging grid-cell for each of the entity AABB corner,
      -- in order to deal with boundary-crossing entities. 
      local aabb = entity.aabb()
      local left, top, right, bottom = unpack(aabb)
      local coords = {
          { left, top },
          { left, bottom },
          { right, bottom },
          { right, top }
        }

      -- We make sure not to store the same entity twice in the same grid-cell
      -- by using the cell's hash-value as a "key".
      local cells = {}
      for _, position in ipairs(coords) do
        local x, y = unpack(position)
        local gx, gy = math.floor(x / size), math.floor(y / size)
        cells[hash(gx, gy)] = true
      end

      -- Build the list of cells (IDs) to which the entity belong. Also, store
      -- the entity in the spatial-hashing table.
      for id, _ in pairs(cells) do
        if not buckets[id] then -- allocate new table for new cells needed
          buckets[id] = {}
        end
        table.insert(buckets[id], entity)
      end
    end
  end

  return buckets
end

function Entities:resolve(entities, filter)
  -- Naive bruteforce O(n^2) collision resolution algorithm (with no
  -- projection at all). As a minor optimization, we scan the pairing
  -- square matrix on the upper (or lower) triangle.
  --
  --     1 2 3 4
  --   1 . x x x
  --   2 . . x x
  --   3 . . . x
  --   4 . . . .
  --
  -- This needs "n(n-1)/2" checks.
  --
  -- http://buildnewgames.com/broad-phase-collision-detection/
  -- http://www.java-gaming.org/index.php?topic=29244.0
  -- http://www.hobbygamedev.com/adv/2d-platformer-advanced-collision-detection/
  -- http://www.wildbunny.co.uk/blog/2011/12/14/how-to-make-a-2d-platform-game-part-2-collision-detection/
  -- http://www.gamedev.net/page/resources/_/technical/game-programming/spatial-hashing-r2697
  for i = 1, #entities - 1 do
    local this = entities[i]
    for j = i + 1, #entities do
      local that = entities[j]
      local collide, h = colliding(this.aabb(), this.delta, that.aabb(), that.delta)
      if collide then
        local resolution, normal = filter(this, that) -- TODO: should also check for "is_alive()"?
        if resolution == 'deflect' then
          deflect(this, h, normal)
          deflect(that, h, normal)
        elseif resolution == 'slide' then
          slide(this, h, normal)
          slide(that, h, normal)
        elseif resolution == 'pass' then
          pass(this, h, normal)
          pass(that, h, normal)
        elseif resolution == 'touch' then
          touch(this, h, normal)
          touch(that, h, normal)
        end -- FIXME: what's the default?
      end
    end
  end
end

-- END OF MODULE ---------------------------------------------------------------

return Entities

-- END OF FILE -----------------------------------------------------------------

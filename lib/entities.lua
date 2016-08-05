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

local function intersect(a, b)
  local ax0, ay0, ax1, ay1 = unpack(a)
  local bx0, by0, bx1, by1 = unpack(b)
  if ax1 < bx0 or ay1 < by0 or ax0 > bx1 or ay0 > by1 then
    return false, {}
  else
    return true, { math.max(ax0, bx0), math.max(ay0, by0), math.min(ax1, bx1), math.min(ay1, by1) }
  end
end

--local function compare(a, b)
--  local delta = a - b
--  if delta < -EPSILON then
--    return -1
--  elseif delta > EPSILON then
--    return 1
--  else
--    return 0
--  end
--end

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

-- TODO: pass the [comparator] and [filter] here? I'm quite positive...
-- TODO: the [filter] function that enables to handle the collision
--       resolution (returning 'touch', 'slide', 'cross' and 'bounce')
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
  --
  local deltas = {}
  -- Update and keep track of the entities that need to be removed.
  --
  -- Since we need to keep the entities relative sorting, we remove "dead"
  -- entities from the back to front. To achive this we "push" the
  -- indices at the front of the to-be-removed list. That way, when
  -- we traverse it we can safely remove the elements as we go.
  local zombies = {}
  for index, entity in ipairs(self.active) do
    local ox, oy = unpack(entity.position)
    entity:update(dt)
    local nx, ny = unpack(entity.position)
    deltas[entity] = { nx - ox, ny - oy } -- TODO: use a "hidden" entity property?
    
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
      self:resolve(entities, deltas, filter)
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
            { right, top },
            { right, bottom }
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

function Entities:resolve(entities, deltas, filter)
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
    local dx, dy = unpack(deltas[this])
    for j = i + 1, #entities do
      local that = entities[j]
      -- TODO: this is too naive! We should project the movement and find the
      --       moment of impact, to solve tunneling issues!
      local collide, area = intersect(this.aabb(), that.aabb())
      if collide then
        local left, top, right, bottom = unpack(area)
        local width, height = right - left, bottom - top
        -- In order to implement DEFLECT and PUSH resolutions we need the
        -- delta of the movement (or the previous position). We compute it
        -- in the [update()] callback.
        local resolution = filter(this, that) -- TODO: should also check for "is_alive()"?
        if resolution == 'deflect' then
          if width < height then
            local delta = dx > 0 and dx - width or dx + width
            this:translate(-delta * 2, 0)
          else
            local delta = dy > 0 and dy - height or dy + height
            this:translate(0, -delta * 2)
          end
        elseif resolution == 'slide' then
          -- Move back along the smaller direction of movement (use deltas?).
          if width < height then
            this:translate(dx > 0 and -width or width, 0)
          else
            this:translate(0, dy > 0 and -height or height)
          end
        elseif resolution == 'touch' then
          this:translate(dx > 0 and -width or width, dy > 0 and -height or height)
        end
      end
    end
  end
end

-- END OF MODULE ---------------------------------------------------------------

return Entities

-- END OF FILE -----------------------------------------------------------------

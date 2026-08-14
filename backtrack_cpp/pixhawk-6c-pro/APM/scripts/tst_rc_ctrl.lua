-- RC Channel 5 -> MANUAL mode if > 1900

local RC_CHANNEL = 5       -- RC input channel number
local THRESHOLD  = 1600    -- threshold for switching
local MODE       = 0       -- 0 = MANUAL mode in ArduPilot

function update()
    local rc_val = rc:get_pwm(RC_CHANNEL)
    if rc_val and rc_val > THRESHOLD then
        vehicle:set_mode(MODE)
    end
    return update, 1000  -- run every 1 second
end

return update()

---@diagnostic disable: param-type-mismatch
---@diagnostic disable: missing-parameter

MODE_MANUAL = 0

-- Scripting 1
EN_SERVO_NUM = 94
VRT_POS_SERVO_NUM = 95
VRT_NEG_SERVO_NUM = 96
HOR_POS_SERVO_NUM = 97
HOR_NEG_SERVO_NUM = 98
HNG_POS_SERVO_NUM = 99
HNG_NEG_SERVO_NUM = 100
HOK_POS_SERVO_NUM = 101
HOK_NEG_SERVO_NUM = 102

-- Winch (separate CAN node, independent drive)
WIN_POS_SERVO_NUM = 103   -- Scripting 10
WIN_NEG_SERVO_NUM = 104   -- Scripting 11

local mavlink_msgs = require("mavlink/mavlink_msgs")

local msg_map = {}

local heartbeat_msgid = mavlink_msgs.get_msgid("HEARTBEAT")
local manual_control_msgid = mavlink_msgs.get_msgid("MANUAL_CONTROL")

msg_map[heartbeat_msgid] = "HEARTBEAT"
msg_map[manual_control_msgid] = "MANUAL_CONTROL"

-- initialize MAVLink rx with buffer depth and number of rx message IDs to register
mavlink.init(10, 2)

-- register message IDs
mavlink.register_rx_msgid(heartbeat_msgid)
mavlink.register_rx_msgid(manual_control_msgid)

local last_hb_time = millis()
local timeout_ms = 2000   -- 1 seconds

-- remember last button states
local last_r_axis = 0
local last_buttons = 0
local last_buttons2 = 0

-- aggregated button state (array 0..31, booleans)
local buttons_state = {}
for i = 0, 31 do
    buttons_state[i] = false
end

function update()
    local msg, chan, timestamp_ms = mavlink.receive_chan()
    if msg then
        local parsed_msg = mavlink_msgs.decode(msg, msg_map)

        if parsed_msg.msgid == manual_control_msgid then
            local buttons  = parsed_msg.buttons  or 0
            local buttons2 = parsed_msg.buttons2 or 0
            local r_axis   = parsed_msg.r        or 0
            last_hb_time = millis()

            -- check for changes
            if r_axis ~= last_r_axis or buttons ~= last_buttons or buttons2 ~= last_buttons2 then
                -- which bits changed?
                local changedr = last_r_axis   ~ r_axis
                local changed  = last_buttons  ~ buttons
                local changed2 = last_buttons2 ~ buttons2

                -- report buttons turned ON/OFF
                for i = 8, 15 do
                    if (changedr & (1 << i)) ~= 0 then
                        local idx   = i - 8
                        buttons_state[idx] = ((r_axis & (1 << i)) ~= 0)
                        local state =  buttons_state[idx] and "ON" or "OFF"
                        gcs:send_text(6, string.format("Button%d %s", idx, state))

                        -- local string_table = {}
                        -- for i, v in ipairs(buttons_state) do
                        --   string_table[i] = tostring(v)
                        -- end
                        -- gcs:send_text(6, table.concat(string_table, ", "))
                    end
                end
                for i = 0, 15 do
                    if (changed & (1 << i)) ~= 0 then
                        local idx   = i + 8
                        buttons_state[idx] = ((buttons & (1 << i)) ~= 0)
                        local state =  buttons_state[idx] and "ON" or "OFF"
                        gcs:send_text(6, string.format("Button%d %s", idx, state))
                    end
                end
                for i = 0, 15 do
                    if (changed2 & (1 << i)) ~= 0 then
                        local state = ((buttons2 & (1 << i)) ~= 0) and "ON" or "OFF"
                        gcs:send_text(6, string.format("Button%d %s", i+8+16, state))
                    end
                end

                -- update last states
                last_r_axis   = r_axis
                last_buttons  = buttons
                last_buttons2 = buttons2
            end

            ---------------------------------------------------
            -- Button actions
            ---------------------------------------------------
            if buttons_state[0] then
                vehicle:set_mode(MODE_MANUAL)
                gcs:send_text(6, "Mode set to MANUAL")
            end


            -- check for heartbeat timeout
            -- if is_timed_out or not arming:is_armed() then
            if not arming:is_armed() then
                -- gcs:send_text(6, "No manual control command for 2 seconds!")
                for i, v in ipairs(buttons_state) do
                  buttons_state[i] = false
                end
            end


            local crunch_en = false
            if buttons_state[2] then
                SRV_Channels:set_output_pwm(VRT_POS_SERVO_NUM, 2000)
                SRV_Channels:set_output_pwm(VRT_NEG_SERVO_NUM, 1000)
                crunch_en = true
            elseif buttons_state[3] then
                SRV_Channels:set_output_pwm(VRT_POS_SERVO_NUM, 1000)
                SRV_Channels:set_output_pwm(VRT_NEG_SERVO_NUM, 2000)
                crunch_en = true
            else
                SRV_Channels:set_output_pwm(VRT_POS_SERVO_NUM, 1000)
                SRV_Channels:set_output_pwm(VRT_NEG_SERVO_NUM, 1000)
            end

            if buttons_state[4] then
                SRV_Channels:set_output_pwm(HOR_POS_SERVO_NUM, 2000)
                SRV_Channels:set_output_pwm(HOR_NEG_SERVO_NUM, 1000)
                crunch_en = true
            elseif buttons_state[5] then
                SRV_Channels:set_output_pwm(HOR_POS_SERVO_NUM, 1000)
                SRV_Channels:set_output_pwm(HOR_NEG_SERVO_NUM, 2000)
                crunch_en = true
            else
                SRV_Channels:set_output_pwm(HOR_POS_SERVO_NUM, 1000)
                SRV_Channels:set_output_pwm(HOR_NEG_SERVO_NUM, 1000)
            end

            if buttons_state[6] then
                SRV_Channels:set_output_pwm(HNG_POS_SERVO_NUM, 2000)
                SRV_Channels:set_output_pwm(HNG_NEG_SERVO_NUM, 1000)
                crunch_en = true
            elseif buttons_state[7] then
                SRV_Channels:set_output_pwm(HNG_POS_SERVO_NUM, 1000)
                SRV_Channels:set_output_pwm(HNG_NEG_SERVO_NUM, 2000)
                crunch_en = true
            else
                SRV_Channels:set_output_pwm(HNG_POS_SERVO_NUM, 1000)
                SRV_Channels:set_output_pwm(HNG_NEG_SERVO_NUM, 1000)
            end

            if buttons_state[9] then
                SRV_Channels:set_output_pwm(HOK_POS_SERVO_NUM, 2000)
                SRV_Channels:set_output_pwm(HOK_NEG_SERVO_NUM, 1000)
                crunch_en = true
            elseif buttons_state[10] then
                SRV_Channels:set_output_pwm(HOK_POS_SERVO_NUM, 1000)
                SRV_Channels:set_output_pwm(HOK_NEG_SERVO_NUM, 2000)
                crunch_en = true
            else
                SRV_Channels:set_output_pwm(HOK_POS_SERVO_NUM, 1000)
                SRV_Channels:set_output_pwm(HOK_NEG_SERVO_NUM, 1000)
            end

            -- Winch: independent drive, does NOT assert the engine/pump enable
            if buttons_state[11] then
                SRV_Channels:set_output_pwm(WIN_POS_SERVO_NUM, 2000)
                SRV_Channels:set_output_pwm(WIN_NEG_SERVO_NUM, 1000)
            elseif buttons_state[12] then
                SRV_Channels:set_output_pwm(WIN_POS_SERVO_NUM, 1000)
                SRV_Channels:set_output_pwm(WIN_NEG_SERVO_NUM, 2000)
            else
                SRV_Channels:set_output_pwm(WIN_POS_SERVO_NUM, 1000)
                SRV_Channels:set_output_pwm(WIN_NEG_SERVO_NUM, 1000)
            end

            if crunch_en then
                SRV_Channels:set_output_pwm(EN_SERVO_NUM, 2000)
            else
                SRV_Channels:set_output_pwm(EN_SERVO_NUM, 1000)
            end
        end
    end

    if millis() - last_hb_time > timeout_ms then
        -- gcs:send_text(6, "SAFETY")
        SRV_Channels:set_output_pwm(VRT_POS_SERVO_NUM, 1000)
        SRV_Channels:set_output_pwm(VRT_NEG_SERVO_NUM, 1000)
        SRV_Channels:set_output_pwm(HOR_POS_SERVO_NUM, 1000)
        SRV_Channels:set_output_pwm(HOR_NEG_SERVO_NUM, 1000)
        SRV_Channels:set_output_pwm(HNG_POS_SERVO_NUM, 1000)
        SRV_Channels:set_output_pwm(HNG_NEG_SERVO_NUM, 1000)
        SRV_Channels:set_output_pwm(HOK_POS_SERVO_NUM, 1000)
        SRV_Channels:set_output_pwm(HOK_NEG_SERVO_NUM, 1000)
        SRV_Channels:set_output_pwm(WIN_POS_SERVO_NUM, 1000)
        SRV_Channels:set_output_pwm(WIN_NEG_SERVO_NUM, 1000)
        SRV_Channels:set_output_pwm(EN_SERVO_NUM, 1000)
    end

    return update, 20
end

return update()

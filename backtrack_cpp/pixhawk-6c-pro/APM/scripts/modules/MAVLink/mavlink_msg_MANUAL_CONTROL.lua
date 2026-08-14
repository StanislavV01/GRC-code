-- mavlink_msg_MANUAL_CONTROL.lua
local MANUAL_CONTROL = {}
MANUAL_CONTROL.id = 69
MANUAL_CONTROL.fields = {
    { "target", "<B" },
    { "x", "<h" },
    { "y", "<h" },
    { "z", "<h" },
    { "r", "<h" },
    { "buttons", "<H" },
    { "buttons2", "<H", nil, true },          -- extension field
    { "enabled_extensions", "<B", nil, true } -- extension field
}
return MANUAL_CONTROL

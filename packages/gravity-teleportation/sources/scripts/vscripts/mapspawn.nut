if(!("Entities" in this)) return;
IncludeScript("sl_gravity");

if (IsMultiplayer())
	IncludeScript("multiplayer");
else
	IncludeScript("singleplayer");

EntFireByHandle(Entities.First(), "RunScriptCode", "::mod_logic()", 0.0, null, null);

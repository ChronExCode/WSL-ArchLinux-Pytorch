#pragma once
// ═══════════════════════════════════════════════════════════════
//  HUD Renderer — Cairo-based HUD drawing
// ═══════════════════════════════════════════════════════════════

struct WaylandApp;

/// Render the full HUD into the given shared memory buffer
void render_hud(WaylandApp& app, void* shm_data);

(function () {
  "use strict";

  const rootId = "app-shell";
  const buttonId = "fullscreen-toggle";
  const fallbackClass = "viewer-maximized";
  const bodyClass = "viewer-fullscreen-active";

  function elements() {
    return {
      root: document.getElementById(rootId),
      button: document.getElementById(buttonId),
    };
  }

  function isActive(root) {
    return Boolean(
      root
      && (document.fullscreenElement === root || root.classList.contains(fallbackClass))
    );
  }

  function resizePlots() {
    const resize = function () {
      if (!window.Plotly || !window.Plotly.Plots) return;
      document.querySelectorAll(".js-plotly-plot").forEach(function (plot) {
        if (plot.offsetParent !== null) window.Plotly.Plots.resize(plot);
      });
    };
    window.requestAnimationFrame(resize);
    window.setTimeout(resize, 140);
    window.setTimeout(resize, 360);
  }

  function syncUi() {
    const current = elements();
    const active = isActive(current.root);
    document.body.classList.toggle(bodyClass, active);
    if (current.button) {
      current.button.textContent = active ? "Exit fullscreen" : "Fullscreen";
      current.button.classList.toggle("button-active", active);
      current.button.setAttribute("aria-pressed", active ? "true" : "false");
      current.button.title = active
        ? "Exit fullscreen (Esc)"
        : "Fill the screen (Esc to exit)";
    }
    resizePlots();
  }

  async function toggleFullscreen() {
    const current = elements();
    if (!current.root) return;

    if (document.fullscreenElement) {
      try {
        await document.exitFullscreen();
      } catch (_error) {
        syncUi();
      }
      return;
    }
    if (current.root.classList.contains(fallbackClass)) {
      current.root.classList.remove(fallbackClass);
      syncUi();
      return;
    }

    if (typeof current.root.requestFullscreen === "function") {
      try {
        await current.root.requestFullscreen();
        return;
      } catch (_error) {
        // Embedded browsers can deny the native API. The CSS fallback still
        // provides a full-window workspace and keeps the exit control visible.
      }
    }
    current.root.classList.add(fallbackClass);
    syncUi();
  }

  document.addEventListener("click", function (event) {
    const target = event.target instanceof Element
      ? event.target.closest("#" + buttonId)
      : null;
    if (!target) return;
    event.preventDefault();
    void toggleFullscreen();
  });

  document.addEventListener("fullscreenchange", syncUi);
  document.addEventListener("keydown", function (event) {
    if (event.key !== "Escape" || document.fullscreenElement) return;
    const current = elements();
    if (current.root && current.root.classList.contains(fallbackClass)) {
      current.root.classList.remove(fallbackClass);
      syncUi();
    }
  });
  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", syncUi, { once: true });
  } else {
    syncUi();
  }
}());

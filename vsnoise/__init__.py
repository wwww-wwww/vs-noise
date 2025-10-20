import vsauto

namespace = "vsnoise"


@vsauto.hookimpl
def loadPlugin():
    import importlib.util
    import logging
    from pathlib import Path

    import vapoursynth as vs

    # Gracefully handling cases where vapoursynth has autoloaded a plugin
    # with the same namespace is strongly encouraged.
    preloaded = next(
        (plugin for plugin in vs.core.plugins() if plugin.namespace == namespace), None
    )
    if preloaded:
        logging.warning(
            f"A plugin at '{Path(preloaded.plugin_path)}' has prevented loading '{__name__}'. Please remove the conflicting package or auto-loaded plugin if you wish to use this version."
        )
        return

    vs.core.std.LoadPlugin(importlib.util.find_spec("vsnoise_ext").origin)

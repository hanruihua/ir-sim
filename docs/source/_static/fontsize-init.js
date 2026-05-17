// Apply the saved text-size preference before the body paints
// (avoids a flash). Paired with the sidebar font-size switcher.
(function () {
    var size = "medium";
    try {
        var s = localStorage.getItem("irsim-fontsize");
        if (s === "small" || s === "medium" || s === "large") size = s;
    } catch (e) {}
    document.documentElement.setAttribute("data-fontsize", size);
})();

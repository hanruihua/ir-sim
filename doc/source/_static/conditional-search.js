// Conditional search button display
document.addEventListener('DOMContentLoaded', function() {
    // Check if we're on the homepage (index page)
    const isHomepage = window.location.pathname.endsWith('/') || 
                      window.location.pathname.endsWith('/index.html') ||
                      window.location.pathname.includes('/index');
    
    // Find the search button in the navbar
    const navbarSearch = document.querySelector('.navbar-search');
    const sidebarSearch = document.querySelector('.sidebar-search');
    
    if (isHomepage) {
        // On homepage: show search in navbar, hide in sidebar
        if (navbarSearch) {
            navbarSearch.style.display = 'block';
        }
        if (sidebarSearch) {
            sidebarSearch.style.display = 'none';
        }
    } else {
        // On other pages: hide search in navbar, show in sidebar
        if (navbarSearch) {
            navbarSearch.style.display = 'none';
        }
        if (sidebarSearch) {
            sidebarSearch.style.display = 'block';
        }
    }
}); 
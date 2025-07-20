// Conditional search button display
document.addEventListener('DOMContentLoaded', function() {
    // Check if we're on the homepage (index page)
    const isHomepage = window.location.pathname.endsWith('/') || 
                      window.location.pathname.endsWith('/index.html') ||
                      window.location.pathname.includes('/index');
    
    if (isHomepage) {
        // On homepage: enable navbar search and hide sidebar search
        const navbarSearch = document.querySelector('.navbar-search');
        const sidebarSearch = document.querySelector('.sidebar-search');
        
        if (navbarSearch) {
            navbarSearch.style.display = 'block';
        }
        if (sidebarSearch) {
            sidebarSearch.style.display = 'none';
        }
        
        // Enable navbar search functionality for homepage
        document.body.classList.add('homepage-search');
    } else {
        // On other pages: completely remove navbar search, keep only sidebar search
        const navbarSearch = document.querySelector('.navbar-search');
        if (navbarSearch) {
            navbarSearch.remove(); // Completely remove the navbar search element
        }
    }
}); 
/**
 * Development Auto-refresh Helper
 * Automatically refreshes the browser when server restarts
 * Only use this in development environment!
 */

(function() {
    // Only enable in development (check if running on localhost or agvc.ui)
    if (!window.location.hostname.includes('localhost') && !window.location.hostname.includes('agvc.ui')) {
        return;
    }

    let lastModified = null;
    const checkInterval = 2000; // Check every 2 seconds
    
    // Check if server is responsive and files have changed
    async function checkForChanges() {
        try {
            const response = await fetch('/static/js/tafl-editor.js', { method: 'HEAD' });
            const currentModified = response.headers.get('last-modified');
            
            if (lastModified && currentModified && lastModified !== currentModified) {
                console.log('📝 Files changed, reloading page...');
                window.location.reload();
            }
            lastModified = currentModified;
        } catch (error) {
            // Server might be restarting
            console.log('⏳ Server unreachable, waiting...');
        }
    }
    
    // Start checking for changes
    setInterval(checkForChanges, checkInterval);
    console.log('🔄 Development auto-refresh enabled (checking every', checkInterval/1000, 'seconds)');
})();
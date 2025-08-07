const { chromium } = require('playwright');

async function checkReteV2Usage() {
  const browser = await chromium.launch();
  const page = await browser.newPage();
  
  try {
    console.log('Checking Rete.js v2 vanilla JS usage...');
    
    // Check the actual CDN files to understand the global objects
    await page.goto('https://cdn.jsdelivr.net/npm/rete@2.0.6/rete.min.js', { timeout: 30000 });
    const reteCore = await page.textContent('body');
    
    // Look for global object definitions
    const globalMatches = reteCore.match(/window\[\w+\]|this\[\w+\]|global\[\w+\]/g);
    console.log('=== RETE CORE GLOBALS ===');
    console.log('Global matches:', globalMatches?.slice(0, 10));
    
    // Check for class definitions
    const classMatches = reteCore.match(/class\s+\w+/g);
    console.log('Class definitions:', classMatches?.slice(0, 10));
    
    // Check area plugin
    await page.goto('https://cdn.jsdelivr.net/npm/rete-area-plugin@2.1.3/rete-area-plugin.min.js', { timeout: 30000 });
    const areaPlugin = await page.textContent('body');
    
    const areaGlobals = areaPlugin.match(/window\[\w+\]|this\[\w+\]|global\[\w+\]/g);
    console.log('\n=== AREA PLUGIN GLOBALS ===');
    console.log('Area globals:', areaGlobals?.slice(0, 10));
    
    // Check connection plugin
    await page.goto('https://cdn.jsdelivr.net/npm/rete-connection-plugin@2.1.4/rete-connection-plugin.min.js', { timeout: 30000 });
    const connectionPlugin = await page.textContent('body');
    
    const connectionGlobals = connectionPlugin.match(/window\[\w+\]|this\[\w+\]|global\[\w+\]/g);
    console.log('\n=== CONNECTION PLUGIN GLOBALS ===');
    console.log('Connection globals:', connectionGlobals?.slice(0, 10));
    
    // Try to find namespace patterns
    console.log('\n=== SEARCHING FOR NAMESPACE PATTERNS ===');
    const namespacePattern = /(\w+)\.(\w+)\s*=/g;
    const namespaces = reteCore.match(namespacePattern);
    console.log('Namespace patterns:', namespaces?.slice(0, 15));
    
    // Look for UMD patterns
    const umdPattern = /typeof exports.*typeof module.*typeof define/g;
    const hasUMD = reteCore.match(umdPattern);
    console.log('Has UMD pattern:', !!hasUMD);
    
  } catch (error) {
    console.error('Error:', error.message);
  } finally {
    await browser.close();
  }
}

checkReteV2Usage();
const { chromium } = require('playwright');
const path = require('path');

async function testReteGlobals() {
  const browser = await chromium.launch();
  const page = await browser.newPage();
  
  // Listen to console logs
  page.on('console', msg => {
    console.log('BROWSER:', msg.text());
  });
  
  try {
    const testFile = path.join(__dirname, 'test-v2.html');
    console.log('Loading test file:', testFile);
    
    await page.goto(`file://${testFile}`);
    await page.waitForTimeout(3000); // Wait for scripts to load
    
    // Get all global objects that might be Rete-related
    const globals = await page.evaluate(() => {
      const reteGlobals = {};
      
      // Check common UMD global names
      const possibleNames = [
        'Rete', 'ReteAreaPlugin', 'ReteConnectionPlugin', 
        'ReteRenderUtils', 'ReteSvgPlugin', 'ReteReactPlugin'
      ];
      
      possibleNames.forEach(name => {
        if (window[name]) {
          reteGlobals[name] = {
            type: typeof window[name],
            keys: Object.keys(window[name]),
            constructors: Object.keys(window[name]).filter(key => 
              typeof window[name][key] === 'function' && /^[A-Z]/.test(key)
            )
          };
        }
      });
      
      return reteGlobals;
    });
    
    console.log('\n=== FOUND RETE GLOBALS ===');
    console.log(JSON.stringify(globals, null, 2));
    
  } catch (error) {
    console.error('Error:', error);
  } finally {
    await browser.close();
  }
}

testReteGlobals();
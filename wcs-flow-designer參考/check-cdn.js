const { chromium } = require('playwright');

async function checkReteCDN() {
  const browser = await chromium.launch();
  const page = await browser.newPage();
  
  try {
    console.log('Checking Rete.js CDN versions...');
    
    // Check jsDelivr for rete package
    await page.goto('https://www.jsdelivr.com/package/npm/rete', { timeout: 30000 });
    await page.waitForTimeout(2000);
    
    // Get the version information
    const content = await page.textContent('body');
    console.log('=== RETE PACKAGE INFO ===');
    console.log(content.substring(0, 2000));
    
    // Check available files
    await page.goto('https://cdn.jsdelivr.net/npm/rete@2/', { timeout: 30000 });
    await page.waitForTimeout(2000);
    
    const files = await page.textContent('body');
    console.log('\n=== RETE v2 FILES ===');
    console.log(files);
    
    // Check area plugin
    await page.goto('https://cdn.jsdelivr.net/npm/rete-area-plugin@2/', { timeout: 30000 });
    await page.waitForTimeout(2000);
    
    const areaFiles = await page.textContent('body');
    console.log('\n=== AREA PLUGIN v2 FILES ===');
    console.log(areaFiles);
    
    // Check connection plugin
    await page.goto('https://cdn.jsdelivr.net/npm/rete-connection-plugin@2/', { timeout: 30000 });
    await page.waitForTimeout(2000);
    
    const connectionFiles = await page.textContent('body');
    console.log('\n=== CONNECTION PLUGIN v2 FILES ===');
    console.log(connectionFiles);
    
  } catch (error) {
    console.error('Error:', error.message);
  } finally {
    await browser.close();
  }
}

checkReteCDN();
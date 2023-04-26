const path = require('path');
const cpy = require('cpy');

(async () => {
    let destination = process.argv.length >= 2 ? process.argv[2] : 'documentation';
    console.log(destination);
    if (!path.isAbsolute(destination)) {
        destination = path.join(process.cwd(), destination);
    }
	await cpy('starter', path.join(destination),
        {
            cwd: __dirname,
            dot: true
        }
    );
	console.log('Your documentation is ready!');
})();